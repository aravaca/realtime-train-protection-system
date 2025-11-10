git add .
git commit -m "graphic fixed ratio"
git push origin main

pip install fastapi uvicorn[standard]
cd tasc
uvicorn server:app --reload --host 127.0.0.1 --port 8000
