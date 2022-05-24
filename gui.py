from tkinter import *
from tkinter import font
from tkinter import filedialog
import sys
import glob
import serial 
import socket
import threading
import time
import subprocess
import json
import os
import signal
from common import ErrorCode
import re

try:
    sys.path.append(glob.glob(os.path.join(sys.path[0],'../PythonAPI/carla/dist/carla-*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64')))[0])
except IndexError:
    pass
import carla

MODE_STRS     = ["0: CARLA without PASTA", "1: PASTA control using LFA6U"]
DFLT_MODE_STR = MODE_STRS[0]
DFLT_MODE     = 0
TOWN_STRS     = ['Town01', 'Town02', 'Town03', 'Town04', 'Town05']
DFLT_TOWN_STR = TOWN_STRS[2]
DFLT_TOWN     = 2
NPC_VEC_MAX   = 10
NPC_PED_NAX   = 50

def is_port_in_use(port):
    import socket
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        return s.connect_ex(('localhost', port)) == 0

class GUI:
    def __init__(self, root):
        self.root = root
        self.mode = DFLT_MODE
        self.availablePorts = []
        self.stop_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

        self.running = False
        self.changing_map = False
        self.log_on = BooleanVar()
        self.disableLS = BooleanVar()
        self.disableLS.set(True)
        self.modeVar = StringVar()
        self.modeVar.set(DFLT_MODE_STR)
        self.townVar = StringVar()
        self.townVar.set(DFLT_TOWN_STR)
        self.filterVar = StringVar()
        self.filterVar.set("vehicle.*")
        self.NVar = StringVar()
        self.NVar.set(str(0))
        self.WVar = StringVar()
        self.WVar.set(str(0))
        self.carlaServerProcess = None
        self.scan_lock = threading.Lock()

        self.topFrame = Frame(self.root)
        self.topFrame.grid(row = 0, column = 0, sticky= "NESW")
        self.configWindow = None

        self.modeLabel       = Label      (self.topFrame, text = "Mode Selection")
        self.labelFont = font.Font(self.modeLabel, self.modeLabel.cget("font"))
        self.labelFont.configure(underline=True)
        self.modeLabel.configure(font=self.labelFont)

        self.controlsLabel   = Label      (self.topFrame, text = "Controls", font=self.labelFont)
        self.modeBox         = OptionMenu (self.topFrame, self.modeVar, *MODE_STRS, command=self.set_mode)
        self.configureButton = Button     (self.topFrame, text="Config", command=self.create_configWindow)
        self.startButton     = Button     (self.topFrame, text="Start", state=NORMAL, command=self.start)
        self.stopButton      = Button     (self.topFrame, text="Stop", state=DISABLED, command=self.stop)
        self.error_label     = Label      (self.topFrame, text="", fg='#f00')
        self.show_log_button = Checkbutton(self.topFrame, text='Show Log',variable=self.log_on, onvalue=True, offvalue=False, command=self.show_log)

        self.log_text = Text(self.root, wrap=WORD)

        self.modeLabel      .grid(row = 0, column = 0, sticky="W")
        self.modeBox        .grid(row = 1, column = 0, rowspan = 2, sticky="W")
        self.show_log_button.grid(row = 3, column = 0, sticky="W")

        self.topFrame.columnconfigure(1, minsize=100)
        self.controlsLabel  .grid(row = 0, column = 1, sticky = "E")
        self.startButton    .grid(row = 1, column = 1, sticky = "E")
        self.configureButton.grid(row = 1, column = 3)
        self.stopButton     .grid(row = 1, column = 2, padx = 10)
        self.error_label    .grid(row = 3, column = 1, columnspan = 30, sticky = "WE")

        self.lfa6uCOMport = StringVar(self.root)
        # can't set tkinter vars outside mainthread
        self.lfa6uCOMport_thread_var = ""
        self.scan_COM_ports()

        self.carlaServerLoc = StringVar(self.root)
        self.carlaServerLoc.set("")

        self.root.columnconfigure(0, minsize=565)

        self.configDict = {}
        # Always add new things to bottom here so if fail from not being in save file doesn't matter
        try:
            with open(os.path.join(sys.path[0], 'GUI_Config.json'), 'r') as file:
                data = file.read()
            self.configDict = json.loads(data)
            self.filterVar.set     (self.configDict['filter'])
            self.disableLS.set     (self.configDict['disableLS'])
            self.carlaServerLoc.set(self.configDict['carlaServer'])
            self.townVar.set       (self.configDict['town'])
            try:
                n = int(self.configDict['npcVec'])
                if n < 0:
                    self.NVar.set("0")
                elif n > NPC_VEC_MAX:
                    self.NVar.set(str(NPC_VEC_MAX))
                else:
                    self.NVar.set  (self.configDict['npcVec'])
            except ValueError:
                self.NVar.set("0")

            try:
                w = int(self.configDict['npcWlk'])
                if w < 0:
                    self.WVar.set("0")
                elif w > NPC_PED_NAX:
                    self.WVar.set(str(NPC_PED_NAX))
                else:
                    self.WVar.set  (self.configDict['npcWlk'])
            except ValueError:
                self.WVar.set("0")

        except Exception as e:
            print("Could not read all of GUI_Config.json file.")

    def show_log(self):
        if self.log_on.get():
            self.log_text.grid(row=10, column=0, columnspan=4, sticky="W")
        else:
            self.log_text.grid_forget()

    def create_configWindow(self):
        if self.configWindow != None:
            self.configWindow.destroy()
            self.configWindow = None
            return
        self.configWindow = Toplevel()
        self.configWindow.title('Configuration')
        self.configWindow.focus_set()

        self.comLabel = Label(self.configWindow, text = "COM Port Settings")
        self.comLabel.grid(row=0, column=0)

        self.comLabel.configure(font=self.labelFont)

        self.rescanButton = Button(self.configWindow, text="Scan", command=self.refresh_COM_ports)
        self.rescanButton.grid(row = 1, column=2, padx=10)

        #self.lfa6uCOMport.set(self.configDict['lfa6u'])

        self.lfa6uLabel = Label     (self.configWindow, text = "LFA6U")
        self.lfa6uComlb = OptionMenu(self.configWindow, self.lfa6uCOMport, *self.availablePorts, command=self.set_COM_port)

        self.lfa6uLabel.grid(row=1, column=0)
        self.lfa6uComlb.grid(row=1, column=1)

        self.otherLabel = Label(self.configWindow, text = "Other Settings", font=self.labelFont)
        self.disableLSButton = Checkbutton(self.configWindow, text='Light Display',variable=self.disableLS, onvalue=False, offvalue=True)
        self.otherLabel.grid     (row=4, column=0, pady=10, sticky="SW")
        self.disableLSButton.grid(row=6, column=0, sticky="W")

        self.filterLabel = Label(self.configWindow, text="Vehicle Filter:")
        self.filterBox = Entry(self.configWindow, textvariable=self.filterVar)
        self.filterLabel.grid(row=7, column=0, sticky="W")
        self.filterBox  .grid(row=7, column=1, sticky="W")

        self.carlaServerLabel = Label(self.configWindow, text="Selected CARLA server:")
        self.carlaServerLabel.grid(row=8, column=0, sticky="W")
        
        self.carlaServerLocLabel = Label(self.configWindow, text="Not set")
        self.carlaServerLocLabel.grid(row=9, column=0, columnspan=30, sticky="W")
        if self.carlaServerLoc.get() != "":
            self.carlaServerLocLabel.configure(text=self.carlaServerLoc.get())
        
        self.selectCarlaServerButton = Button(self.configWindow, text="Select Server EXE", command=self.select_carla_filename)
        self.selectCarlaServerButton.grid(row=10, column=0, sticky="W")

        self.townLabel = Label     (self.configWindow, text="Map (changes on restart)")
        self.townBox   = OptionMenu(self.configWindow, self.townVar, *TOWN_STRS)
        self.townChangeButton = Button(self.configWindow, text="Apply Map", command=self.change_town)
        self.townLabel       .grid(row=11, column=0, sticky="W")
        self.townBox         .grid(row=11, column=1)
        self.townChangeButton.grid(row=11, column=2)

        self.npcLabel = Label(self.configWindow, text="Traffic:", font=self.labelFont)
        vcmd = (self.root.register(self.validateInt), '%P' , '%W')
        self.npcNLabel = Label(self.configWindow, text="Vehicles (0-10):")
        self.npcWLabel = Label(self.configWindow, text="Pedestrians (0-50):")
        self.NEntry    = Entry(self.configWindow, textvariable=self.NVar, validate = 'all', validatecommand = vcmd, width=3)
        self.WEntry    = Entry(self.configWindow, textvariable=self.WVar, validate = 'all', validatecommand = vcmd, width=3)
        self.npcLabel .grid(row=12, column=0, sticky="w")
        self.npcNLabel.grid(row=13, column=0)
        self.NEntry   .grid(row=13, column=1, sticky="w")
        self.npcWLabel.grid(row=14, column=0)
        self.WEntry   .grid(row=14, column=1, sticky="w")

        self.dfltButton = Button(self.configWindow, text="Default", command=self.default)
        self.dfltButton.grid(row = 100, column=2)

        self.saveButton = Button(self.configWindow, text="Save", command=self.save)
        self.saveButton.grid(row = 100, column=3)

    def validateInt(self, value_if_allowed, widget_name):
        if value_if_allowed:
            try:
                if not str.isdigit(value_if_allowed):
                    return False
                v = int(value_if_allowed)
                if widget_name == ".!toplevel.!entry2": # vehicle box
                    maxV = NPC_VEC_MAX
                elif widget_name == ".!toplevel.!entry3": # pedestrian box
                    maxV = NPC_PED_NAX
                else: # I don't know why end up here but appears to only happen on load so doesn't matter: values still bound in GUI
                    maxV = 999
                if v <= maxV:
                    return True
                else:
                    return False
            except ValueError as e:
                return False
        else:
            return True

    def validateFloat(self, action, index, value_if_allowed,
                       prior_value, text, validation_type, trigger_type, widget_name):
        if value_if_allowed:
            try:
                v = float(value_if_allowed)
                if v >= 0 and v <= 180:
                    return True
                return False
            except ValueError:
                return False
        else:
            return True

    def change_town(self):
        self.stop()
        try:
            client = carla.Client('127.0.0.1', 2000)
            threading.Thread(target = self.change_town_thread, args = [client], daemon=True).start()
            self.changing_map = True
            self.startButton.configure(state=DISABLED)
        except Exception as e:
            self.error_label.configure(text="Failed to connect to CARLA server and change world.")
        
    def change_town_thread(self, client):
        try:
            client.load_world(self.townVar.get())
        except Exception as e:
            self.error_label.configure(text="Failed to connect to CARLA server and change world.")
        finally:
            self.changing_map = False
            self.check_if_can_start()

    def select_carla_filename(self):
        server = filedialog.askopenfilename(filetypes=[("CARLAUE4.exe", "*.exe")])
        if os.path.basename(server) != "CarlaUE4.exe":
            self.carlaServerLoc.set("")
            self.carlaServerLocLabel.configure(text="Invalid EXE", fg="#f00")
        else:
            self.carlaServerLoc.set(server)
            self.carlaServerLocLabel.configure(text=server, fg="#000")

    def save(self):
        self.configDict['disableLS']   = self.disableLS.get()
        self.configDict['filter']      = self.filterVar.get()
        self.configDict['carlaServer'] = self.carlaServerLoc.get()
        self.configDict['town']        = self.townVar.get()
        self.configDict['npcVec']      = self.NVar.get()
        self.configDict['npcWlk']      = self.WVar.get()
        try:
            with open(os.path.join(sys.path[0],'GUI_Config.json'), 'w') as fp:
                json.dump(self.configDict, fp)
            self.configWindow.destroy()
            self.configWindow.update()
            self.configWindow = None
        except Exception as e:
            self.errorLabel = Label(self.configWindow, text = "Error could not save file: " + str(e) , fg='#f00')
            self.errorLabel.grid(row = 10, column=0, columnspan=9)
            return
        try:
            if self.configDict['carlaServer'] != "" :
                serverFolder = os.path.dirname(os.path.abspath(self.configDict['carlaServer']))
                configFile = os.path.join(serverFolder, 'CarlaUE4', 'Config', 'DefaultEngine.ini')
                with open(configFile, 'r') as file :
                    filedata = file.read()

                filedata = re.sub("Town0.", self.configDict['town'], filedata, flags = re.M)

                with open(configFile, 'w') as file:
                    file.write(filedata)
                
        except Exception as e:
            self.errorLabel = Label(self.configWindow, text = "Error could not find CARLA server config file from exe location: " + str(e) , fg='#f00')
            self.errorLabel.grid(row = 10, column=0, columnspan=9)


    def default(self):
        self.disableLS.set(True)
        self.filterVar.set("vehicle.*")

    def start(self):
        self.startButton.configure(state=DISABLED)
        self.running = True
        self.log_text.delete('1.0', END)
        runCmd = sys.executable + " " + os.path.join(sys.path[0],"main.py") + " --mode " + str(self.mode)
        if self.mode == 1 :
            if self.lfa6uCOMport.get() != "":
                runCmd += " --lfa6u " + self.lfa6uCOMport.get()
        if self.disableLS.get():
            runCmd += " --disable_light_state_display"
        runCmd += " --filter \"" + self.filterVar.get() + "\""
        threading.Thread(target = self.run_thread, args = [runCmd], daemon=True).start()

    def send_kill(self):
        self.stop_sock.sendto(bytes("kill", 'ascii'), ("127.0.0.1", 3000))
        self.stop_sock.sendto(bytes("kill", 'ascii'), ("127.0.0.1", 3002))
        self.stop_sock.sendto(bytes("kill", 'ascii'), ("127.0.0.1", 3003))
        self.stop_sock.sendto(bytes("kill", 'ascii'), ("127.0.0.1", 3004))

    def stop(self):
        # pulling default from others if change 3003 default param need to change it here
        self.send_kill()
        #TODO os.kill complains about permissions, can't get groupid + CARLAUE4 launches CARLAUE4-Win64-..... only have PID for first
        time.sleep(1) # TODO bit of hack not actually checking process exited just delaying 
        # NOTE button stuff needs to come last: running this function on quit so want above to happen but below may error on quit so just chu
        self.running = False
        self.startButton.configure(state=NORMAL)
        self.stopButton.configure(state=DISABLED)
        self.error_label.configure(text="")

    def start_CARLA(self):
        # start CARLA server
        if self.carlaServerLoc.get() != "":
            print("Found path to CARLA; trying to start it")
            try:
                client = carla.Client('127.0.0.1', 2000)
                client.set_timeout(2.0)
                world = client.get_world()
                success = True
            except RuntimeError as e:
                success = False
            if not success:
                self.log_text.insert(INSERT, "Starting CARLA server \n")
                subprocess.Popen(os.path.abspath(self.carlaServerLoc.get()))
                retry = 0
                while not success:
                    try:
                        client = carla.Client('127.0.0.1', 2000)
                        client.set_timeout(2.0)
                        world = client.get_world()
                        success = True
                    except RuntimeError as e:
                        pass
                    retry += 1
                if not success:
                    self.error_label.configure(text="Failed to start and connect to CARLA server.")
                    self.running = False
                    self.startButton.configure(state=NORMAL)
                    return
                self.log_text.insert(INSERT, "Done; confirmed can connect to server\n")
            else:
                self.log_text.insert(INSERT, "Found a running server can connect to\n")

    def run_thread(self, runCmd):
        # run main.py
        try:
            self.log_text.insert(INSERT, runCmd +"\n")
            out = subprocess.check_output(runCmd, stderr=subprocess.STDOUT)
            self.error_label.configure(text="Need to stop before starting again.")
            self.stopButton.configure(state=NORMAL)
        except subprocess.CalledProcessError as exc:
            errText= "Run failed:"
            if exc.returncode == ErrorCode.COM.value:
                errText += " could not open COM port."
            if exc.returncode == ErrorCode.CARLA.value:
                errText += " could not connect to CARLA."
            if exc.returncode == ErrorCode.FILTER.value:
                errText += " vehicle filter too restrictive."
            self.error_label.configure(text=errText)
            self.running = False
            self.startButton.configure(state=NORMAL)
            out = "FAILED with code " + str(exc.returncode) + "\n" + str(exc.output)
        self.log_text.insert(INSERT, out)
        cmd = sys.executable + " " + os.path.join(sys.path[0],"spawn_npc.py")
        cmd += " -n " + self.NVar.get()
        cmd += " -w " + self.WVar.get()
        self.spawnNPCproc = subprocess.Popen(cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

    def set_COM_port(self, *args):
        if self.running:
            return

        self.check_if_can_start()

    def refresh_COM_ports(self):
        self.scan_COM_ports()
        self.lfa6uComlb.destroy()

        self.lfa6uComlb = OptionMenu(self.configWindow, self.lfa6uCOMport, *self.availablePorts, command=self.set_COM_port)
        self.lfa6uComlb.grid(row=1, column=1)

    def com_port_scan_thread(self, ports):
        availablePorts = []
        for port in ports:
            success = False
            retry = 0
            while not success and retry < 20:
                try:
                    s =  serial.serial_for_url(port, stopbits=serial.STOPBITS_TWO, timeout=0.1, writeTimeout = 0.1)
                    success = True
                except OSError as e:
                    #print(e)
                    pass
                retry += 1
            if success: # can at least open port
                success = False
                retry = 0
                try: # try outside while now -> if couldn't open port and now get errors give up
                    while not success and retry < 20:
                        if retry >= 10:
                            s.write(bytes("BPS1152 2 0 0\r", "ascii"))
                            s.baudrate = 115200
                            resp = s.read(10)
                        #print("Sending ? to port", port, "attempt", retry +1)
                        try:
                            s.write(b"?\r")
                        except serial.serialutil.SerialTimeoutException:
                            retry += 1
                            #print("write timeout")
                            continue
                        resp = s.read(1000)
                        #print("after read resp:", resp)
                        if bytes("LFA6U", "ascii") in resp:
                            with self.scan_lock:
                                self.lfa6uCOMport_thread_var = port
                            success = True
                        #print(retry, resp)
                        retry += 1
                    s.close()
                    availablePorts.append(port)
                except OSError as e:
                    #print(e)
                    pass

        with self.scan_lock:
            for i in availablePorts:
                self.availablePorts.append(i)

    def scan_COM_ports(self, num_threads=4):
        self.lfa6uCOMport.set("")
        self.lfa6uCOMport_thread_var = ""
        self.availablePorts = []
        threads = []
        for i in range(num_threads):
            ports = ['COM%s' % (i + 1) for i in range(i, 256, num_threads)]
            t = threading.Thread(target = self.com_port_scan_thread, args = [ports], daemon=True)
            t.start()
            threads.append(t)

        for t in threads:
            t.join()
            
        self.lfa6uCOMport.set(self.lfa6uCOMport_thread_var)

        self.availablePorts.sort()

        self.check_if_can_start()

    def clear_COM(self):
        self.lfa6uLabel.grid_forget()
        self.lfa6uComlb.grid_forget()

    def set_mode(self, mode):
        self.mode = int(mode.split(":")[0])
        self.check_if_can_start()

    def check_if_can_start(self):
        self.error_label.configure(text="")
        if   (self.mode == 1):
            if self.lfa6uCOMport.get() != "":
                self.startButton.configure(state=NORMAL)
            else: # No port for lfa6u
                self.error_label.configure(text="No LFA6U found")
                self.startButton.configure(state=DISABLED)
        elif self.mode == 0: # always can start in mode 0 as no PASTA communication setup
            self.startButton.configure(state=NORMAL)
        if  self.startButton['state'] == 'normal' and (self.running or self.changing_map):
            self.startButton.configure(state=DISABLED)
            self.error_label.configure(text="Need to stop before starting again.")

root = Tk()
root.title("CARLA - PASTA control")
root.resizable(False, False)
gui = GUI(root)
gui.start_CARLA()
root.mainloop()
try: # kill manual_control if it is running; don't care about exceptions here as exitting anyways
    gui.send_kill()
    time.sleep(1) # TODO bit of hack not actually checking process exited just delaying 
    print("Stopping. Trying to stop CARLA:")
    subprocess.run('taskkill /f /im CARLAUE4*', shell=True) #TODO trying stdout=DEVNULL or PIPE etc. suppresses output but makes it so taskkill doesn't work
except:
    pass