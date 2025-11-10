# Modifiche:
############
# - Aggiunta la disattivazione del wifi alla fine della funzione "download_and_install_update_if_available"
# La funzione "connect_wifi" ha ora un'attesa massima di circa 10 secondi e ritorna un valore True o False
# - Aggiunta la funzione "set_time" per aggiornare l'orologio interno (RTC) all'ora di Greenwich (GMT)
# Da fare:
############
# 1. ....
#
# Note:
############
# 1. Se per il collegamento internet si usa un mobile hotspot, attivare e configurare l'hotspot dopo aver spento
# il WiFi perchè questo modo il telefono deve mostrare tutte le opzioni come la scelta della "Band AP" 2.4/5.0 GHz
# è necessario selezionare la banda 2.4 GHz

import network
import urequests
import os
import json
import machine
# from time import sleep
import time
import ntptime
import socket
import struct

sta_if = None

class OTAUpdater:
    """ This class handles OTA updates. It connects to the Wi-Fi, checks for updates, downloads and installs them."""
    def __init__(self, ssid, password, repo_url, filename):
        self.filename = filename
        self.ssid = ssid
        self.password = password
        self.repo_url = repo_url
        if "www.github.com" in self.repo_url :
            print(f"Updating {repo_url} to raw.githubusercontent")
            self.repo_url = self.repo_url.replace("www.github","raw.githubusercontent")
        elif "github.com" in self.repo_url:
            print(f"Updating {repo_url} to raw.githubusercontent'")
            self.repo_url = self.repo_url.replace("github","raw.githubusercontent")            
        self.version_url = self.repo_url + 'main/version.json'
        print(f"version url is: {self.version_url}")
        self.firmware_url = self.repo_url + 'main/' + filename

        # get the current version (stored in version.json)
        if 'version.json' in os.listdir():    
            with open('version.json') as f:
                self.current_version = int(json.load(f)['version'])
            print(f"Current device firmware version is '{self.current_version}'")

        else:
            self.current_version = 0
            # save the current version
            with open('version.json', 'w') as f:
                json.dump({'version': self.current_version}, f)
            
    def connect_wifi(self):
        """ Connect to Wi-Fi."""
        global sta_if
        
        sta_if = network.WLAN(network.STA_IF)
        sta_if.active(True)
        sta_if.connect(self.ssid, self.password)
        count = 0
        while not sta_if.isconnected():
            print('.', end="")
            count += 1
            time.sleep(0.25)
            if count > 40:
                print('Connection timeout')
                return False
        else:
            print(f'Connected to WiFi, IP is: {sta_if.ifconfig()[0]}')
            self.set_time() # Imposta RTC interno
            return True
        
    def fetch_latest_code(self)->bool:
        """ Fetch the latest code from the repo, returns False if not found."""
        
        # Fetch the latest code from the repo.
        response = urequests.get(self.firmware_url)
        if response.status_code == 200:
            print(f'Fetched latest firmware code, status: {response.status_code}, -  {response.text}')
    
            # Save the fetched code to memory
            self.latest_code = response.text
            return True
        
        elif response.status_code == 404:
            print(f'Firmware not found - {self.firmware_url}.')
            return False

    def update_no_reset(self):
        """ Update the code without resetting the device."""

        # Save the fetched code and update the version file to latest version.
        with open('latest_code.py', 'w') as f:
            f.write(self.latest_code)
        
        # update the version in memory
        self.current_version = self.latest_version

        # save the current version
        with open('version.json', 'w') as f:
            json.dump({'version': self.current_version}, f)
        
        # free up some memory
        self.latest_code = None

        # Overwrite the old code.
#         os.rename('latest_code.py', self.filename)

    def update_and_reset(self):
        """ Update the code and reset the device."""

        print(f"Updating device... (Renaming latest_code.py to {self.filename})", end="")

        # Overwrite the old code.
        os.rename('latest_code.py', self.filename)  

        # Restart the device to run the new code.
        print('Restarting device...')
        machine.reset()  # Reset the device to run the new code.
        
    def check_for_updates(self):
        """ Check if updates are available."""
        
        # Connect to Wi-Fi
        if not self.connect_wifi():
            return False

        print(f'Checking for latest version... on {self.version_url}')
        response = urequests.get(self.version_url)
        
        data = json.loads(response.text)
        
        print(f"data is: {data}, url is: {self.version_url}")
        print(f"data version is: {data['version']}")
        # Turn list to dict using dictionary comprehension
#         my_dict = {data[i]: data[i + 1] for i in range(0, len(data), 2)}
        
        self.latest_version = int(data['version'])
        print(f'latest version is: {self.latest_version}')
        
        # compare versions
        newer_version_available = True if self.current_version < self.latest_version else False
        
        print(f'Newer version available: {newer_version_available}')    
        return newer_version_available
    
    def download_and_install_update_if_available(self):
        """ Check for updates, download and install them."""
        if self.check_for_updates():
            if self.fetch_latest_code():
                self.update_no_reset() 
                self.update_and_reset()     
        else:
            print('No new updates available.')
        
        sta_if.active(False) # MRB Test: è opportuno spegnere il wifi?

    def set_time(self):
        """ Update internal RTC """
        try:
            ntptime.settime()
            print("NTP Time: ", time.localtime())
        except:
            print("Errore nella sincronizzazione NTP")
        
