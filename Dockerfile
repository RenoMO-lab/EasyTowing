FROM python:3.11-slim-bookworm

ENV PYTHONDONTWRITEBYTECODE=1 \
    PYTHONUNBUFFERED=1

WORKDIR /app

COPY pyproject.toml README.md ./
COPY easytowing ./easytowing

RUN pip install --no-cache-dir ".[postgres]" \
    && useradd --create-home --shell /usr/sbin/nologin easytowing \
    && mkdir -p /var/lib/easytowing/artifacts \
    && chown -R easytowing:easytowing /app /var/lib/easytowing

USER easytowing

EXPOSE 8000

CMD ["python", "-m", "easytowing.demo_server", "--host", "0.0.0.0", "--port", "8000"]
