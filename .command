git add .
git commit -m "update readme/ui"
git push origin main

pip install fastapi uvicorn[standard]
uvicorn server:app --reload --host 127.0.0.1 --port 8000
