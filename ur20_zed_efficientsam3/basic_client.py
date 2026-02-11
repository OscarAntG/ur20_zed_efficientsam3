import requests

try:
    print("Connecting to SAM3 server...")
    response = requests.get("http://localhost:8000/")

    if response.status_code == 200:
        print("Connection successful, response:")
        print(response.json())
    else:
        print(f"\nConnection failed, status: {response.status_code}")

except Exception as e:
    print(f"\nCritical Failure: {e}")