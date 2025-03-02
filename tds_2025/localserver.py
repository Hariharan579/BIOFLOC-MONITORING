import os
import urllib.request
import http
import pandas as pd
import re
from time import sleep
from datetime import datetime

base = "http://192.168.137.158/"

def transfer(my_url):   #use to send and receive data
    try:
        n = urllib.request.urlopen(base + my_url).read()
        n = n.decode("utf-8")
        return n
    except http.client.HTTPException as e:
        return e

# Specify the absolute path for the Excel file
excel_file_path = r"C:\Users\Hxtreme\Documents\Arduino\tds_2025\sensor_data2.xlsx"

# Create an empty list to store data
data_list = []

# Create a function to split and save data to Excel
def save_to_excel(te, gsr, ppg,ph):
    data_list.append([te,gsr, ppg,ph])
    df = pd.DataFrame(data_list, columns=['TEMPERATURE', 'tds', 'turbidity','PH'])
    df.to_excel(excel_file_path, index=False)

ct = 0
while True:
    res = transfer(str(ct))
    response = str(res)
    print(response)
    
    # Split the received data
    values = response.split('-')
    if len(values) == 4:
        te, gsr, ppg,ph = values
        save_to_excel(te, gsr, ppg,ph)
    
    sleep(1)
