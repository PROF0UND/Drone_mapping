#!/usr/bin/env bash
set -euo pipefail

SERVICE_NAME="telemetry-logger.service"
SERVICE_PATH="/etc/systemd/system/${SERVICE_NAME}"
SOURCE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

if [[ "${EUID}" -ne 0 ]]; then
  echo "Run this installer with sudo:"
  echo "  sudo bash ${SOURCE_DIR}/install_telemetry_service.sh"
  exit 1
fi

cp "${SOURCE_DIR}/${SERVICE_NAME}" "${SERVICE_PATH}"
systemctl daemon-reload
systemctl enable "${SERVICE_NAME}"

echo "Installed and enabled ${SERVICE_NAME}."
echo "Start it now with:"
echo "  sudo systemctl start ${SERVICE_NAME}"
echo
echo "Check status/logs with:"
echo "  systemctl status ${SERVICE_NAME}"
echo "  journalctl -u ${SERVICE_NAME} -f"
