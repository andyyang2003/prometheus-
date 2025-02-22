#import paho.mqtt.client as mqtt
import csv
import time
import paho.mqtt.client as paho
from paho import mqtt
import re
import os
import datetime
import logging
import keyboard
#log file details
log_file = "sensor_collect.log"
logging.basicConfig(filename=log_file, level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s') 
# MQTT broker details 
mqtt_broker = "8a6d3324d5d542f682b67e26bbc1baa4.s1.eu.hivemq.cloud" # url for cluster 
mqtt_port = 8883  # Or 8883 for TLS
mqtt_topic = "esp32/dht22_sensor"  # The topic you're publishing to
mqtt_username = "ayang11" 
mqtt_password = "gVXI1250KD$"

# CSV file details
csv_file = "sensor_data.csv"
csv_header = ["Timestamp"]

# CREATE DATA BUFFER TO SAVE TO INSTEAD OF WRITING 
data_buffer = []

def on_connect(client, userdata, flags, rc, properties=None):    
    log_message = f"Connected with result code {str(rc)}"
    logging.info(log_message)
    client.subscribe(mqtt_topic)  # Subscribe to the topic
    log_message = f"Subscribed to topic: {mqtt_topic}" 
    logging.info(log_message)

def on_message(client, userdata, msg):
    timestamp = time.strftime("%Y-%m-%d %H:%M:%S")  # Get current timestamp
    data = msg.payload.decode() # decode string
    log_message = f"logged data at {timestamp}"
    logging.info(log_message)   
    parsed_data = parseData(data) 
    if parsed_data:  # Check if parsing was successful
        data_buffer.append({"Timestamp" : timestamp, **parsed_data}) # append dictionary with key = timestamp value = parsed data
        # add keys in parsed_data as headers
        for key in parsed_data: 
            if key not in csv_header:
                csv_header.append(key)

        with open(csv_file, 'a', newline='') as csvfile:  # Open in append mode ('a')
            writer = csv.writer(csvfile)

            # Write header if the file is empty (only once)
            if csvfile.tell() == 0:  # Check if file is empty
                writer.writerow(csv_header)  # Write header row

            row_data = [timestamp]  # Start with timestamp
            for key in csv_header[1:]: #add value based on the header
                row_data.append(parsed_data.get(key)) #append value, if does not exist, append none
            writer.writerow(row_data) #write the row
def save_to_csv():
    # if theres stuff in teh data buffer
    global data_buffer, csv_header, csv_file
    if data_buffer:
        try:
            # check if file exists
            file_exists = os.path.isfile(csv_file)
            # open csv file in append mode
            with open(csv_file, 'a', newline='') as csvfile:
                writer = csv.DictWriter(csvfile, fieldnames=csv_header) # create writer object with filename and headers
                if not file_exists: 
                    writer.writeheader() #ONLY IF FILE HASNT BEEN CREATED WRITE HEADERS
                writer.writerows(data_buffer) # write all buffered data at once
            log_message = f"Data saved to {csv_file} at {datetime.datetime.now().strftime('%Y-%m-%d %H:%M:%S')}"
            logging.info(log_message)() 
            data_buffer = []  # clear buffer
        except Exception as e:
            error_message = f"Error saving to CSV: {e}"
            logging.info(error_message)
            


def parseData(inputString):
    try:
        #tx:%0.2f t:%0.2f h:%0.2f d:%0.2f lt:%0.2f ln:%0.2f gas:%d is the payload format
        data = {}
        
        pairs = re.findall(r"(\w+):([\d.-]+)", inputString)
        for key, value in pairs:
            try:
                # convert value to float / int
                if "." in value: # if it has a decimal, make float
                    data[key] = float(value)
                else:
                    data[key] = int(value) 
            #catch error and just keep it as normal stirng
            except ValueError:
                data[key] = value
        return data
    except Exception as e:
        print(f"Error parsing {e}")
        return None
    
# ------- SET UP CONNECTION TO MQTT BROKER
client = paho.Client(client_id="", userdata=None, protocol=paho.MQTTv5)
client.on_connect = on_connect
client.on_message = on_message # collect message

client.tls_set(tls_version=mqtt.client.ssl.PROTOCOL_TLS)
client.username_pw_set(mqtt_username, mqtt_password) #Set username and password

client.connect(mqtt_broker, mqtt_port) #Connect to broker

client.loop_start()
last_save = time.time()
try:
    while True:
        if keyboard.is_pressed('space'):
            break
        current_save = time.time()
        print("last save" + str(last_save))
        print("current save" + str(current_save))
        if current_save - last_save > 5: 
            last_save = current_save
            save_to_csv()
            print("saved")
        time.sleep(1) # wait 5 minutes 
except KeyboardInterrupt:
    logging.info("user exit")
    print("exit")
finally:
    save_to_csv()
    client.loop_stop()
    client.disconnect()
    logging.info("loop stopped")
    print("loop stop")



