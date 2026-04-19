#!/usr/bin/env python3
"""通过 J-Link RTT 自动抓取 pitch 标定导出协议并落盘成 CSV。"""

from __future__ import annotations

import argparse
import datetime as dt
import re
import socket
import subprocess
import sys
import time
from pathlib import Path


# 这里统一定义 ANSI 控制码清洗正则，目的是 RTT 日志默认带颜色控制序列，脚本若不先去色就无法稳定识别导出协议前缀。
ANSI_ESCAPE_RE = re.compile(r"\x1B\[[0-?]*[ -/]*[@-~]")
# 这里统一定义日志级别前缀清洗正则，目的是固件通过 `LOGINFO/LOGWARNING` 输出时会额外带上 `I:/W:/E:`，CSV 协议解析前必须先把它们剥掉。
LOG_LEVEL_PREFIX_RE = re.compile(r"^\s*(?:[IWE]:)\s*")

# 这里把 RTT 导出协议的几个固定前缀集中成常量，目的是固件和脚本约定靠这些前缀同步，一旦未来协议调整，只需要在一处改动即可。
EXPORT_BEGIN_PREFIX = "PITCH_CALI_EXPORT_BEGIN|"
EXPORT_HEADER_PREFIX = "PITCH_CALI_EXPORT_HEADER|"
EXPORT_ROW_PREFIX = "PITCH_CALI_EXPORT_ROW|"
EXPORT_END_PREFIX = "PITCH_CALI_EXPORT_END|"
EXPORT_ABORT_PREFIX = "PITCH_CALI_EXPORT_ABORT|"


class PitchCaliExportSession:
    """管理单次 pitch 标定导出会话的本地 CSV 落盘状态。"""

    def __init__(self, output_dir: Path, keep_partial: bool) -> None:
        # 这里缓存输出目录和 `.partial` 保留策略，目的是 begin/abort/end 三个阶段都要围绕同一份落盘配置做一致处理。
        self.output_dir = output_dir
        self.keep_partial = keep_partial
        # 这里把文件句柄和路径状态都初始化成空，目的是脚本启动后要先等待固件 begin 标记，不能提前假定已经有有效会话。
        self.file_handle = None
        self.partial_path: Path | None = None
        self.final_path: Path | None = None
        # 这里记录当前已经实际写入 CSV 的数据行数，目的是结束时要和固件上报的 `rows` 做一次完整性比对。
        self.row_count = 0
        # 这里记录表头是否已经写入，目的是导出协议必须先收到 header 再写 row，若顺序异常需要直接告警而不是默默生成坏文件。
        self.header_written = False

    def reset(self) -> None:
        """清空当前会话的内存状态。"""
        # 这里在会话结束或异常清理后统一清空状态，目的是下一轮标定必须从一个完全干净的上下文开始，不能沿用旧路径和旧计数。
        self.file_handle = None
        self.partial_path = None
        self.final_path = None
        self.row_count = 0
        self.header_written = False

    def begin(self) -> Path:
        """收到 begin 标记后创建新的 `.partial` 文件。"""
        # 这里先确保输出目录存在，目的是脚本默认把导出文件放到 `gimbal/build/pitch_cali_exports`，首次使用时目录通常还不存在。
        self.output_dir.mkdir(parents=True, exist_ok=True)
        # 这里用主机本地时间戳生成文件名，目的是当前固件侧没有可信 RTC，最终文件名必须由上位机侧保证唯一且可追溯。
        timestamp = dt.datetime.now().strftime("%Y-%m-%d_%H%M%S")
        self.final_path = self.output_dir / f"pitch_cali_{timestamp}.csv"
        self.partial_path = self.output_dir / f"pitch_cali_{timestamp}.csv.partial"
        # 这里显式用换行模式创建 `.partial` 文件，目的是 CSV 在 Linux/Windows 环境下都要保持单一的 `\n` 行尾，避免后续处理再遇到双重换行。
        self.file_handle = self.partial_path.open("w", encoding="utf-8", newline="\n")
        self.row_count = 0
        self.header_written = False
        return self.partial_path

    def write_header(self, header_line: str) -> None:
        """写入 CSV 表头。"""
        if self.file_handle is None:
            raise RuntimeError("export session has not been started")
        # 这里把固件给出的 header 原样写入本地文件，目的是列顺序和列名都应以固件当前导出定义为准，而不是由脚本侧私自硬编码。
        self.file_handle.write(f"{header_line}\n")
        self.file_handle.flush()
        self.header_written = True

    def write_row(self, row_line: str) -> None:
        """写入一行 CSV 数据。"""
        if self.file_handle is None:
            raise RuntimeError("export session has not been started")
        if not self.header_written:
            raise RuntimeError("export header has not been written")
        # 这里把每一行数据原样追加到 CSV，目的是避免脚本再次解析浮点字符串造成格式漂移，确保文件内容与 RTT 导出的原始值完全一致。
        self.file_handle.write(f"{row_line}\n")
        self.file_handle.flush()
        self.row_count += 1

    def finish(self, expected_rows: int | None) -> Path:
        """收到 end 标记后关闭 `.partial` 并原子改名成正式 CSV。"""
        if self.file_handle is None or self.partial_path is None or self.final_path is None:
            raise RuntimeError("export session is incomplete")
        # 这里先把文件刷盘并关闭，再做重命名，目的是保证最终用户看到的 `.csv` 一定是完整落盘后的稳定文件。
        self.file_handle.flush()
        self.file_handle.close()
        self.file_handle = None
        if expected_rows is not None and expected_rows != self.row_count:
            # 这里保留行数不一致告警但仍然生成文件，目的是现场偶发丢行时用户至少还能拿到当前抓到的数据，同时也能立刻看到完整性异常。
            print(
                f"[pitch_cali_export] warning: expected {expected_rows} rows but captured {self.row_count} rows",
                file=sys.stderr,
            )
        self.partial_path.replace(self.final_path)
        final_path = self.final_path
        self.reset()
        return final_path

    def abort(self) -> None:
        """收到 abort 标记或脚本异常退出时清理 `.partial`。"""
        if self.file_handle is not None:
            # 这里先关闭当前打开的文件句柄，目的是无论最终是删除还是保留 `.partial`，都不能让未关闭文件句柄拖住后续清理。
            self.file_handle.close()
            self.file_handle = None
        if self.partial_path is not None and self.partial_path.exists() and not self.keep_partial:
            # 这里默认删除 `.partial`，目的是用户要的是“完成后得到正式 CSV”，未完成文件若默认保留会更容易被误当成有效标定结果。
            self.partial_path.unlink()
        self.reset()


def normalize_log_line(raw_line: str) -> str:
    """去掉 RTT 的颜色控制码和日志级别前缀，只保留业务文本。"""
    # 这里先去掉 ANSI 控制序列，目的是 `JLinkRTTClient` 读出的原始文本会把颜色转义一并带回来，直接匹配协议前缀会失败。
    cleaned_line = ANSI_ESCAPE_RE.sub("", raw_line).strip()
    # 这里再剥掉 `I:/W:/E:` 级别前缀，目的是固件导出协议挂在 `LOGINFO/LOGWARNING` 上，脚本只需要纯净的协议正文。
    cleaned_line = LOG_LEVEL_PREFIX_RE.sub("", cleaned_line)
    return cleaned_line.strip()


def wait_for_tcp_port(host: str, port: int, timeout_s: float) -> bool:
    """等待本地 RTT Telnet 端口可连通。"""
    # 这里统一用短连接轮询端口是否起来，目的是 `JLinkGDBServerCLExe` 的 RTT Telnet 端口不是瞬时可用，脚本必须先等服务准备好再启动 RTT client。
    deadline = time.time() + timeout_s
    while time.time() < deadline:
        try:
            with socket.create_connection((host, port), timeout=1.0):
                return True
        except OSError:
            time.sleep(0.2)
    return False


def parse_expected_rows(line: str) -> int | None:
    """从 end/abort 标记里提取 `rows=` 字段。"""
    # 这里按最小解析策略只关心 `rows=` 这一项，目的是脚本完成落盘只需要做完整性比对，不需要把整条协议拆成复杂结构。
    for part in line.split("|"):
        if part.startswith("rows="):
            try:
                return int(part.split("=", 1)[1])
            except ValueError:
                return None
    return None


def build_parser(repo_root: Path) -> argparse.ArgumentParser:
    """构建命令行参数解析器。"""
    default_output_dir = repo_root / "gimbal" / "build" / "pitch_cali_exports"
    parser = argparse.ArgumentParser(
        description="监听 J-Link RTT 中的 pitch 标定导出协议，并自动保存成 CSV 文件。"
    )
    # 这里默认拉起一份临时 J-Link GDB Server，目的是当前仓库已经固定用 J-Link + RTT 调试，脚本应尽量做到开箱即用，不要求用户先手工起服务。
    parser.add_argument(
        "--no-spawn-server",
        action="store_true",
        help="不启动新的 J-Link GDB Server，直接连接已存在的 RTT Telnet 端口。",
    )
    # 这里把常用的 J-Link 目标参数都开放成命令行选项，目的是同一脚本后续也能兼容不同探头序号、目标型号或 RTT 端口配置。
    parser.add_argument("--device", default="STM32F407IG", help="J-Link 目标设备名。")
    parser.add_argument("--interface", default="SWD", help="J-Link 调试接口，例如 SWD。")
    parser.add_argument("--speed", default="4000", help="J-Link 接口速度，单位 kHz。")
    parser.add_argument("--gdb-port", type=int, default=2331, help="J-Link GDB Server 监听端口。")
    parser.add_argument("--rtt-telnet-port", type=int, default=19021, help="RTT Telnet 端口。")
    parser.add_argument("--select", default="", help="可选的 J-Link 选择参数，例如 USB=123456789。")
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=default_output_dir,
        help=f"导出 CSV 的输出目录，默认是 {default_output_dir}",
    )
    parser.add_argument(
        "--keep-partial",
        action="store_true",
        help="若导出被 abort 或脚本异常退出，保留 `.partial` 文件用于排查。",
    )
    return parser


def start_gdb_server(args: argparse.Namespace) -> subprocess.Popen[str]:
    """按当前参数启动一份临时 J-Link GDB Server。"""
    command = [
        "JLinkGDBServerCLExe",
        "-device",
        args.device,
        "-if",
        args.interface,
        "-speed",
        args.speed,
        "-port",
        str(args.gdb_port),
        "-nohalt",
        "-nogui",
    ]
    if args.select:
        # 这里允许用户显式选择探头，目的是实验室里多把 J-Link 并存时，脚本必须能稳定绑定到目标设备而不是随机抢一把。
        command.extend(["-select", args.select])
    # 这里把 stdout/stderr 都接回脚本，目的是若服务器启动失败，用户需要直接看到 J-Link 给出的原始诊断信息，而不是只得到“端口没起来”的模糊错误。
    process = subprocess.Popen(
        command,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
        encoding="utf-8",
        errors="replace",
    )
    return process


def start_rtt_client(args: argparse.Namespace) -> subprocess.Popen[str]:
    """启动 J-Link RTT Client 并把输出交给脚本逐行解析。"""
    command = ["JLinkRTTClient", "-RTTTelnetPort", str(args.rtt_telnet_port)]
    # 这里把 RTT Client 的输出改成管道读取，目的是脚本必须逐行解析 begin/header/row/end 协议，不能只把终端输出简单重定向到文件。
    process = subprocess.Popen(
        command,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
        encoding="utf-8",
        errors="replace",
        bufsize=1,
    )
    return process


def terminate_process(process: subprocess.Popen[str] | None) -> None:
    """尽量温和地结束子进程，必要时再升级为 kill。"""
    if process is None or process.poll() is not None:
        return
    # 这里先尝试 `terminate`，目的是 J-Link 工具通常能在收到普通终止信号后自己清理连接，不需要一上来就强杀。
    process.terminate()
    try:
        process.wait(timeout=3.0)
    except subprocess.TimeoutExpired:
        # 只有在进程确实不响应时才升级成 `kill`，目的是尽量减少 J-Link 残留调试会话和端口占用。
        process.kill()
        process.wait(timeout=3.0)


def main() -> int:
    repo_root = Path(__file__).resolve().parent.parent
    parser = build_parser(repo_root)
    args = parser.parse_args()

    session = PitchCaliExportSession(args.output_dir, args.keep_partial)
    gdb_server_process: subprocess.Popen[str] | None = None
    rtt_client_process: subprocess.Popen[str] | None = None

    try:
        if not args.no_spawn_server:
            print("[pitch_cali_export] starting J-Link GDB Server...", file=sys.stderr)
            gdb_server_process = start_gdb_server(args)
            if not wait_for_tcp_port("127.0.0.1", args.rtt_telnet_port, timeout_s=10.0):
                # 这里在 RTT 端口长时间未就绪时直接读取并抛出 GDB Server 日志，目的是用户最需要看到的是设备名、连线或探头错误等底层原因。
                server_output = ""
                terminate_process(gdb_server_process)
                if gdb_server_process.stdout is not None:
                    try:
                        server_output = gdb_server_process.stdout.read()
                    except Exception:
                        server_output = ""
                raise RuntimeError(
                    "J-Link RTT server did not become ready in time.\n"
                    f"{server_output.strip()}"
                )

        print("[pitch_cali_export] waiting for pitch calibration export...", file=sys.stderr)
        rtt_client_process = start_rtt_client(args)
        if rtt_client_process.stdout is None:
            raise RuntimeError("failed to capture RTT client stdout")

        for raw_line in rtt_client_process.stdout:
            # 这里同时把原始 RTT 输出继续转发到终端，目的是用户在保存 CSV 的同时仍能实时看到固件侧当前处于开始、导出还是完成阶段。
            sys.stdout.write(raw_line)
            sys.stdout.flush()

            parsed_line = normalize_log_line(raw_line)
            if not parsed_line:
                continue

            if parsed_line.startswith(EXPORT_BEGIN_PREFIX):
                if session.file_handle is not None:
                    # 若固件异常重复发 begin，就先把旧 `.partial` 清掉再重新开始，目的是避免两轮会话的数据混进同一份文件。
                    session.abort()
                partial_path = session.begin()
                print(f"[pitch_cali_export] capture started -> {partial_path}", file=sys.stderr)
                continue

            if parsed_line.startswith(EXPORT_HEADER_PREFIX):
                if session.file_handle is None:
                    print("[pitch_cali_export] warning: header received before begin", file=sys.stderr)
                    continue
                session.write_header(parsed_line[len(EXPORT_HEADER_PREFIX) :])
                continue

            if parsed_line.startswith(EXPORT_ROW_PREFIX):
                if session.file_handle is None:
                    print("[pitch_cali_export] warning: row received before begin", file=sys.stderr)
                    continue
                session.write_row(parsed_line[len(EXPORT_ROW_PREFIX) :])
                continue

            if parsed_line.startswith(EXPORT_ABORT_PREFIX):
                expected_rows = parse_expected_rows(parsed_line)
                print(
                    f"[pitch_cali_export] export aborted after {expected_rows if expected_rows is not None else session.row_count} rows",
                    file=sys.stderr,
                )
                session.abort()
                # 中止只代表本轮标定没有完成，脚本继续等待下一次成功导出，目的是现场重复试标时无需每次重新启动抓取工具。
                continue

            if parsed_line.startswith(EXPORT_END_PREFIX):
                expected_rows = parse_expected_rows(parsed_line)
                final_path = session.finish(expected_rows)
                print(f"[pitch_cali_export] export finished -> {final_path}", file=sys.stderr)
                return 0

        if rtt_client_process.poll() not in (0, None):
            raise RuntimeError(f"RTT client exited with code {rtt_client_process.returncode}")
        raise RuntimeError("RTT client closed before a completed pitch calibration export was captured")
    except KeyboardInterrupt:
        # 用户主动终止时同样走统一清理路径，目的是不能把还没闭合的 `.partial` 和 J-Link 子进程残留在后台。
        print("[pitch_cali_export] interrupted by user", file=sys.stderr)
        session.abort()
        return 130
    finally:
        if rtt_client_process is not None and rtt_client_process.poll() is None:
            terminate_process(rtt_client_process)
        if gdb_server_process is not None and gdb_server_process.poll() is None:
            terminate_process(gdb_server_process)


if __name__ == "__main__":
    sys.exit(main())
