git add .
git commit -m "graphic update"
git push origin main

pip install fastapi uvicorn[standard]
uvicorn server:app --reload --host 127.0.0.1 --port 8000
