git add .
git commit -m "start weather update"
git push

pip install fastapi uvicorn[standard]
uvicorn server:app --reload --host 127.0.0.1 --port 8000
