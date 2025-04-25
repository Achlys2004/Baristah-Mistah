# flask_control_server.py
from flask import Flask, request
from datetime import datetime

app = Flask(__name__)

@app.route('/control', methods=['POST'])
def control():
    try:
        data = request.json
        print(f"[{datetime.now()}] ✅ Received control data: {data}")
        return {'status': 'received', 'message': 'Control signal logged successfully'}
    except Exception as e:
        print(f"[{datetime.now()}] ❌ Error: {e}")
        return {'status': 'error', 'message': str(e)}, 500

if __name__ == '__main__':
    app.run(host='0.0.0.0', port=5000, debug=True)
