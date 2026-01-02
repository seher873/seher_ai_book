# Multi-stage build for optimized production Docker image
# Stage 1: Builder - Install dependencies
FROM python:3.10-slim as builder

# Set working directory
WORKDIR /app

# Install build dependencies
RUN apt-get update && \
    apt-get install -y --no-install-recommends \
    gcc \
    g++ \
    build-essential \
    && rm -rf /var/lib/apt/lists/*

# Copy requirements and install Python packages
COPY requirements.txt .
RUN pip install --no-cache-dir --upgrade pip && \
    pip install --no-cache-dir --user -r requirements.txt

# Stage 2: Runtime - Minimal production image
FROM python:3.10-slim

# Set working directory
WORKDIR /app

# Create non-root user for security first
RUN useradd -m -u 1000 appuser

# Copy Python packages from builder to appuser's home
COPY --from=builder /root/.local /home/appuser/.local

# Copy application code
COPY . .

# Change ownership of all files to appuser
RUN chown -R appuser:appuser /app /home/appuser/.local

# Set environment variables
ENV PYTHONPATH=/app \
    PORT=7860 \
    PATH=/home/appuser/.local/bin:$PATH \
    PYTHONUNBUFFERED=1

# Expose port
EXPOSE 7860

# Switch to non-root user
USER appuser

# Health check
HEALTHCHECK --interval=30s --timeout=10s --start-period=40s --retries=3 \
    CMD python -c "import urllib.request; urllib.request.urlopen('http://localhost:7860/health').read()" || exit 1

# Run the application
CMD ["uvicorn", "main:app", "--host", "0.0.0.0", "--port", "7860"]
