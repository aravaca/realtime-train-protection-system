git add .
git commit -m "air brk update"
git push

pip install fastapi uvicorn[standard]
uvicorn server:app --reload --host 127.0.0.1 --port 8000
