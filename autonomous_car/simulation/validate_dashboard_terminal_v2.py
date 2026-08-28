"""Static regression for the private-LAN dashboard terminal integration."""

from pathlib import Path

from dashboard_terminal_hmi import DASHBOARD_TERMINAL_HMI
from settings_page_shell import SETTINGS_PAGE_SHELL


def _require(condition, message):
    if not condition:
        raise AssertionError(message)


def main():
    hmi = DASHBOARD_TERMINAL_HMI.decode("utf-8")
    shell = SETTINGS_PAGE_SHELL.decode("utf-8")
    installer = Path("scripts/install_dashboard_terminal.sh").read_text(encoding="utf-8")
    runner = Path("scripts/run_dashboard_terminal.sh").read_text(encoding="utf-8")

    for token in (
        "#view-system .grid",
        "id='swing-terminal-panel'",
        "터미널 열기",
        "http://${host}:7681/",
        "AUTH REQUIRED",
        "차량이 정지되어 있고",
    ):
        _require(token in hmi, f"dashboard terminal HMI contract missing: {token}")

    # SETTINGS_PAGE_SHELL is the already-concatenated bytes payload sent to the
    # browser. The Python symbol name is intentionally not present after that
    # concatenation, so validate the rendered terminal contract instead.
    for token in (
        "swing-terminal-panel",
        "swing-terminal-open",
        "터미널 열기",
        "http://${host}:7681/",
    ):
        _require(token in shell, f"settings shell missing terminal UI: {token}")

    for token in (
        "apt-get install -y ttyd",
        "swing-dashboard-terminal.service",
        "User=$TARGET_USER",
        "EnvironmentFile=$ENV_FILE",
        "chmod 0600 \"$ENV_FILE\"",
        "systemctl enable --now swing-dashboard-terminal.service",
    ):
        _require(token in installer, f"terminal installer contract missing: {token}")

    for token in (
        "--interface \"$INTERFACE\"",
        "--credential \"$CREDENTIAL\"",
        "--writable",
        "--check-origin",
        "--max-clients \"$MAX_CLIENTS\"",
        "/bin/bash -l",
    ):
        _require(token in runner, f"ttyd runtime contract missing: {token}")

    print("Dashboard terminal V2 regression: PASS")
    print(
        {
            "dashboard_system_button": "PASS",
            "private_lan_interface": "PASS",
            "basic_auth": "PASS",
            "gnss_user_service": "PASS",
            "separate_ttyd_process": "PASS",
        }
    )


if __name__ == "__main__":
    main()
