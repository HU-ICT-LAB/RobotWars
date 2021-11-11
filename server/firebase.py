"""Database test"""
import firebase_admin
from firebase_admin import credentials, db
from datetime import datetime


def setup_firebase():
    cred = credentials.Certificate("service_account_key.json")
    firebase_admin.initialize_app(cred, {
        'databaseURL': 'https://robotwars-6fc0a-default-rtdb.europe-west1.firebasedatabase.app/'
    })


def get_all_data():
    return db.reference("all_findings/").get()


def retrieve_policy_from_database():
    return


def write_findings_to_database(policy_reference, findings):
    data_reference = db.reference("all_findings/")
    data_reference.push().set({
        "time": datetime.now().strftime("%d/%m/%Y_%H:%M:%S"),
        "policy": policy_reference,
        "findings": findings
    })


if __name__ == "__main__":
    setup_firebase()
    print(get_all_data())
    write_findings_to_database("test", {"turn_right": "ok", "turn_left": "ok"})
    print(get_all_data())
