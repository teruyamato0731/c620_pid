FROM mcr.microsoft.com/devcontainers/base:jammy

RUN apt-get update && apt-get install -y \
  python3-pip \
  python3-venv \
  bash-completion \
  && rm -rf /var/lib/apt/lists/*

RUN python3 -m pip install --no-cache-dir -U \
  platformio \
  jinja2 \
  intelhex \
  future \
  prettytable \
  six \
  jsonschema
RUN pio system completion install bash --path /etc/bash_completion.d/platformio
