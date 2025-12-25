from fastapi import FastAPI
from app.main import app  # import your FastAPI app

def list_post_paths(app: FastAPI):
    print("POST endpoints in your app:\n")
    for route in app.routes:
        if "POST" in route.methods:
            print(f"{route.path}")

if __name__ == "__main__":
    list_post_paths(app)
