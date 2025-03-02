import os
import urllib.request
import http
import pandas as pd
import re
from time import sleep
from datetime import datetime

import pickle
filename = 'model1.sav'
loaded_model = pickle.load(open(filename, 'rb'))

base = "http://192.168.137.239/"

def transfer(my_url):   #use to send and receive data
    try:
        n = urllib.request.urlopen(base + my_url).read()
        n = n.decode("utf-8")
        return n
    except http.client.HTTPException as e:
        return e

# Specify the absolute path for the Excel file

# Create an empty list to store data
data_list = []

ct = 0
while True:
    res = transfer(str(ct))
    response = str(res)
    print(response)
    
    # Split the received data
    values = response.split(',')
    if len(values) == 4:
        te, gsr, ppg,ph = values
        
        #save_to_excel(v1, c1, v2, c2)
        reports = [[te, gsr, ppg,ph]]
        ppg=ppg*-1
        predicted = loaded_model.predict(reports)
        print(predicted)
        ft=predicted[0]
        if ft in "BAD":
            ct=2
        else:
            ct=1
        conn = urllib.request.urlopen("https://api.thingspeak.com/update?api_key=1Z4GBPZ7ZZEV8NCW&field1="+str(te)+"&field2="+str(gsr)+"&field3="+str(ppg)+"&field4="+str(ph)+"&field5="+str(ft))        #response = conn.read()
        #print ("http status code=%s" % (conn.getcode()))
        #print("")
    
    sleep(1)

