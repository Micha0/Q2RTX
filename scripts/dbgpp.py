import gdb

class vec3_tPrinter:
    def __init__(self, val):
        self.val = val

    def to_string(self):
        return "{ " + str(self.val[0]) + ", " + str(self.val[1]) + ", " + str(self.val[2]) + " }"

class btVectorPrinter:
    def __init__(self, val):
        self.oval = val
        self.val = val["m_floats"]

    def to_string(self):
        return "{ " + str(self.val[0]) + ", " + str(self.val[1]) + ", " + str(self.val[2]) + ", " + str(self.val[3]) + " }"

class btAlignedObjectArrayPrinter:
    def __init__(self, val):
        self.val = val
        self.size = val["m_size"]
        self.capacity = val["m_capacity"]
        self.data = val["m_data"]

    def to_string(self):
        return "{ size: " + str(self.size) + ", capacity: " + str(self.capacity) + " }"


def lookup_type(val):
    if(str(val.type) == 'vec3_t'):
        return vec3_tPrinter(val)
    if(str(val.type) == 'btVector3'):
        return btVectorPrinter(val)
    if(str(val.type) == 'btAlignedObjectArray'):
        return btAlignedObjectArrayPrinter(val)
    return None

gdb.pretty_printers.append(lookup_type)