# SBD Metallic Debris Collector Project
# Stanley Black and Decker
# Georgia Institute of Technology
# created by Tyler Brown


#This file is used to create a GUI for the user that will allow the user to
#control the robot from a computer attached to the robot's network

import Tkinter as tk
import tkFont
from PIL import ImageTk
from PIL import Image
import socket

global img

#destination IP and port
UDP_IP = "10.42.0.1"
UDP_PORT = 4200

#create socket
udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)


#send message to robot
def send_message(message):

    #use socket to send the deisred message
    udp_socket.sendto(message, (UDP_IP, UDP_PORT))

#callback for stop button
def stop_callback():

    #send stop message
    send_message("stop")

#callback form start message
def start_callback():

    #send start message
    send_message("start")

def populate_window(window):

    #set up the logo on the top of the GUI
    img = Image.open("sbd.png")
    img_container = ImageTk.PhotoImage(img)
    canvas = tk.Canvas(window, width=300, height=40)
    canvas.pack(side='top')
    canvas.configure(bg='black')
    canvas.create_image(70, 22, anchor='center', image=img_container)
    canvas.image = img_container

    #create the buttons to be added to the GUI

    #create a font to use on the button text
    helv36 = tkFont.Font(family='Helvetica', size=15, weight=tkFont.BOLD)

    #create a frame to keep the buttons in
    button_container = tk.Frame(window)

    #start button
    start_button = tk.Button(button_container, text = "Start", height=3,
        width=10, font=helv36, command=start_callback)
    start_button.pack(side='top', expand='yes')
    start_button.configure(bg='green', relief='flat')
    start_button.place(relx=0.25, rely=0.5, anchor='center')

    #stop button
    stop_button = tk.Button(button_container, text = "Stop", height=3,
        width=10, font=helv36, command=stop_callback)
    stop_button.pack(side='top', expand='yes')
    stop_button.configure(bg='red', relief='flat')
    stop_button.place(relx=0.75, rely=0.5, anchor='center')

    button_container.pack(fill='both', expand='yes')


#create the tkinter window
window = tk.Tk()
window.title("Stanley Black and Decker RMDC")
window.geometry('300x200')
window.wm_iconbitmap('gatech.ico')

#populate the GUI window
populate_window(window)



window.mainloop()