import os
from threading import Thread
from main import app
from uvicorn import Config, Server

if __name__ == "__main__":
    config = Config(app, host="0.0.0.0", port=int(os.getenv("PORT", 8000)))
    server = Server(config)
    server.run()