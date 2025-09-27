
import logging

class multimeter:

    def get_title(self):
        return self.title

    def get_read_val(self):
        logging.debug("get_read_val() connected, reading ... ")
        read_val = self.instr.read()
        logging.debug("get_read_val() reading "+str(read_val))
        return read_val

    def read_stb(self):
        logging.debug("read_stb() reading status")
        self.stb = self.instr.read_stb()

    def blank_display(self):
        logging.debug("blank_display not implemented for this instrument")


class HP3458A(multimeter):

    def __init__(self, resource_manager, resource_name, title='3458A'):
        self.title = title
        logging.debug(self.title+' init started')
        self.rm = resource_manager
        self.rn = resource_name
        self.instr =  self.rm.open_resource(self.rn)
        self.instr.timeout = 2000000
        self.instr.clear()
        self.instr.write("RESET")
        self.instr.write("END ALWAYS")
        self.instr.write("OFORMAT ASCII")
        self.instr.write("BEEP")
        logging.info("ID? -> "+self.instr.query("ID?"))

    def config_NPLC(self, NPLC):
        logging.debug(self.title+" config_NPLC")
        self.instr.write("NPLC "+str(NPLC))

    def config_NDIG(self, NDIG):
        logging.debug(self.title+" config_NDIG")
        self.instr.write("NDIG "+str(NDIG))

    def config_DCV(self, RANG):
        logging.debug(self.title+" config_DCV")
        self.instr.write("DCV "+str(RANG))

    def blank_display(self):
        logging.debug(self.title+" blank_display")
        self.instr.write("ARANGE ON")
        self.instr.write("DISP MSG,\"                 \"")
        #self.instr.visalib.sessions[self.instr.session].interface.ibloc()

    def config_trigger_auto(self):
        logging.debug(self.title+" config_trigger_auto")
        self.instr.write("TARM AUTO")

    def config_trigger_hold(self):
        logging.debug(self.title+" config_trigger_hold")
        self.instr.write("TARM HOLD")

    def trigger_once(self):
        logging.debug(self.title+' triggered once')
        self.instr.write("TARM SGL")

    def acal_DCV(self):
        logging.debug(self.title+' ACAL DCV started')
        timout_memory = self.instr.timeout
        self.instr.timeout = 2
        try:
            self.instr.write("ACAL DCV")
        except Exception:
            pass
        self.instr.timeout = timout_memory


    def acal_ALL(self):
        logging.debug(self.title+' ACAL ALL started')
        timout_memory = self.instr.timeout
        self.instr.timeout = 2000
        try:
            self.instr.write("ACAL")
        except Exception:
            pass
        self.instr.timeout = timout_memory

    def is_readable(self):
        logging.debug(self.title+' is_readable() started')
        self.read_stb()
        logging.debug(self.title+' stb is '+str(self.stb))
        readable = self.stb & 0b10000000
        return readable

    def is_ready(self):
        self.read_stb()
        ready = self.stb & 0b00010000
        return ready

    def get_int_temp(self):
        temp = self.instr.query("TEMP?")
        return temp

    def get_cal_param(self, number):
        logging.debug(self.title+' get_cal_param() called')
        param = self.instr.query("CAL? "+str(number))
        return param

    def config_pt100(self):
        logging.debug(self.title+' config_pt100 called')
        self.instr.write("OHMF 100")
        self.instr.write("MATH CRTD85")

    def config_OHMF(self, RANG):
        logging.debug(self.title+' config_ohmf called')
        self.instr.write("OHMF "+str(RANG))
        self.instr.write("OCOMP ON")
        self.instr.write("DELAY 1")


if __name__ == "__main__":
    import pyvisa
    rm = pyvisa.ResourceManager()
    print('opening', 'GPIB0::15::INSTR')
    inst = rm.open_resource('GPIB0::15::INSTR')
    inst.clear()
    inst.write("RESET")
    #print(inst.query("*IDN?"))
    #dm = HP3458A()