# EA3121 Robotics Lab 02 - Activity 3 map server
#
# Standard library only, so there is no pip step and no wheel cache:
# the image is python + ~20 KB of lab code.

FROM python:3.12-alpine

WORKDIR /app

COPY server.py .
COPY static/ ./static/

# Unbuffered so `docker logs -f` shows each incoming scan live
ENV PYTHONUNBUFFERED=1 \
    MAP_HOST=0.0.0.0 \
    MAP_PORT=8080

EXPOSE 8080

# Run unprivileged - nothing here needs root
USER nobody

HEALTHCHECK --interval=30s --timeout=3s --start-period=5s \
  CMD wget -qO- http://127.0.0.1:8080/api/health || exit 1

CMD ["python", "server.py"]
