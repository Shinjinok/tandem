#
# PyDroneCAN DSDL parser demo.
# Read the package docstrings for details.
#

# pydronecan supports Python 2.7 and Python 3.x
from __future__ import print_function
from dronecan import dsdl

# Root namespace directories:
NAMESPACE_DIRS = ['/home/sjo/tandem/modules/DroneCAN/namespace_a/', '/home/sjo/tandem/modules/DroneCAN/namespace_b/','/home/sjo/tandem/modules/DroneCAN/DSDL/ardupilot/gnss']

# Where to look for the referenced types (standard UAVCAN types):
SEARCH_DIRS = ['/home/sjo/tandem/modules/DroneCAN/DSDL/uavcan']

# Run the DSDL parser:
types = dsdl.parse_namespaces(NAMESPACE_DIRS, SEARCH_DIRS)

# Print what we got
ATTRIBUTE_INDENT = ' ' * 3

def print_attributes(fields, constants):
    for a in fields:
        print(ATTRIBUTE_INDENT, a.type.full_name, a.name, end='  # ')
        # Add a bit of relevant information for this type:
        if a.type.category == a.type.CATEGORY_COMPOUND:
            print('DSDL signature: 0x%08X' % a.type.get_dsdl_signature())
        if a.type.category == a.type.CATEGORY_ARRAY:
            print('Max array size: %d' % a.type.max_size)
        if a.type.category == a.type.CATEGORY_PRIMITIVE:
            print('Bit length: %d' % a.type.bitlen)
    for a in constants:
        print(ATTRIBUTE_INDENT, a.type.full_name, a.name, '=', a.value, '#', a.init_expression)

for t in types:
    print(t.full_name)
    if t.kind == t.KIND_MESSAGE:
        print_attributes(t.fields, t.constants)
    elif t.kind == t.KIND_SERVICE:
        print_attributes(t.request_fields, t.request_constants)
        print(ATTRIBUTE_INDENT, '---')
        print_attributes(t.response_fields, t.response_constants)