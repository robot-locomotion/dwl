import dwl
import numpy as np
import os


# Construct an instance of the YamlWrapper class, which wraps the C++ class.
yaml = dwl.YamlWrapper()
fpath = str(os.path.dirname(os.path.abspath(__file__)))
yaml.setFile(fpath + '/../example.yaml')


# Declaring the namespace, if you don't have it please use $ns = []
ns = ['global_ns', 'variable_ns']

print 'Parsing the example.yaml file'
print ns[0] + ':'
print '  ' + ns[1] + ':'

# Reading a bool data
read, value = yaml.readBool('bool',  ns)
if read:
  print '    bool:', value

# Reading an int data
read, value = yaml.readInt('int', ns)
if read:
  print '    int:', value

# Reading a double data
read, value = yaml.readDouble('double', ns)
if read:
  print '    double:', value

# Reading a string data
read, value = yaml.readString('string', ns)
if read:
  print '    string:', value

# Reading a double vector data
read, vec = yaml.readDoubleList('double_vector', ns)
array = []
for i in range(0,vec.size()):
  array.append(vec[i])
if read:
  print '    double_vector:', array

# Reading a string vector data
read, vec = yaml.readStringList('string_vector', ns)
array = []
for i in range(0,vec.size()):
  array.append(vec[i])
if read:
  print '    string_vector:', array

# Reading an array 2d
array2d = np.array([0.,0.])
read = yaml.readArray2d(array2d,'vector_2d', ns)
if read:
  print '    array_2d:', array2d

# Reading an array 3d
array3d = np.array([0.,0.,0.])
read = yaml.readArray3d(array3d,'vector_3d', ns)
if read:
  print '    array_3d:', array3d