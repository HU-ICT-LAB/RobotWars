"""Alle functies die te maken hebben met het schrijven en ophalen van gegevens uit de AWS database."""
import mysql.connector
import configparser
import pandas as pd

def data_selection(samplesize: int, seed=None):
    """Fetches a given amount of rows from the AWS data storage.
    If a seed is given, AWS' random number generator seed is
    set to this value."""
    config = configparser.ConfigParser()  # Credentials komen uit een untracked bestand.
    config.read("credentials.cfg")
    host, user, password, db = config['MySQL']['host'], config['MySQL']['user'], config['MySQL']['passwd'], config['MySQL']['db_name']
    # Onthoud, queries alleen voor intern gebruik, buiten intern gebruik is dit een serieus veiligheidsrisico.
    conn = mysql.connector.connect(host, user, password, db)

