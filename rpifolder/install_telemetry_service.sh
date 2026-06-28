#!/usr/bin/env bash
set -euo pipefail

SERVICE_NAME="telemetry-logger.service"
SERVICE_PATH="/etc/systemd/system/${SERVICE_NAME}"
SOURCE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SERVICE_USER="${TELEMETRY_SERVICE_USER:-${SUDO_USER:-profound}}"

if [[ "${EUID}" -ne 0 ]]; then
  echo "Run this installer with sudo:"
  echo "  sudo bash ${SOURCE_DIR}/install_telemetry_service.sh"
  exit 1
fi

if ! id "${SERVICE_USER}" >/dev/null 2>&1; then
  echo "Service user '${SERVICE_USER}' does not exist."
  echo "Set TELEMETRY_SERVICE_USER or run with sudo from the target user account."
  exit 1
fi

SERVICE_HOME="$(getent passwd "${SERVICE_USER}" | cut -d: -f6)"
TELEMETRY_SCRIPT="${TELEMETRY_SCRIPT:-${SERVICE_HOME}/telemetry_logger.py}"

if [[ ! -f "${TELEMETRY_SCRIPT}" ]]; then
  echo "Telemetry script not found at '${TELEMETRY_SCRIPT}'."
  echo "Copy telemetry_logger.py there first, or set TELEMETRY_SCRIPT=/path/to/telemetry_logger.py."
  exit 1
fi

escape_sed_replacement() {
  printf '%s' "$1" | sed -e 's/[|&]/\\&/g'
}

SOURCE_DIR_ESCAPED="$(escape_sed_replacement "${SOURCE_DIR}")"
SERVICE_USER_ESCAPED="$(escape_sed_replacement "${SERVICE_USER}")"
SERVICE_HOME_ESCAPED="$(escape_sed_replacement "${SERVICE_HOME}")"
TELEMETRY_SCRIPT_ESCAPED="$(escape_sed_replacement "${TELEMETRY_SCRIPT}")"

sed \
  -e "s|User=profound|User=${SERVICE_USER_ESCAPED}|" \
  -e "s|WorkingDirectory=/home/profound/mapping|WorkingDirectory=${SOURCE_DIR_ESCAPED}|" \
  -e "s|/home/profound/telemetry_logger.py|${TELEMETRY_SCRIPT_ESCAPED}|" \
  -e "s|/home/profound/.local/bin|${SERVICE_HOME_ESCAPED}/.local/bin|" \
  "${SOURCE_DIR}/${SERVICE_NAME}" > "${SERVICE_PATH}"

systemctl daemon-reload
systemctl enable "${SERVICE_NAME}"

echo "Installed and enabled ${SERVICE_NAME}."
echo "Service user: ${SERVICE_USER}"
echo "Service directory: ${SOURCE_DIR}"
echo "Telemetry script: ${TELEMETRY_SCRIPT}"
echo "Start it now with:"
echo "  sudo systemctl start ${SERVICE_NAME}"
echo
echo "Check status/logs with:"
echo "  systemctl status ${SERVICE_NAME}"
echo "  journalctl -u ${SERVICE_NAME} -f"
