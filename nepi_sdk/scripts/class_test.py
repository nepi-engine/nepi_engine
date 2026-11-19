#
# Copyright (c) 2025 Numurus <https://www.numurus.com>.
#
# This file is part of nepi engine (nepi_engine) repo
# (see https://github.com/nepi-engine/nepi_engine)
#
# License: NEPI Engine repo source-code and NEPI Images that use this source-code
# are licensed under the "Numurus Software License",
# which can be found at: <https://numurus.com/wp-content/uploads/Numurus-Software-License-Terms.pdf>
#
# Redistributions in source code must retain this top-level comment block.
# Plagiarizing this software to sidestep the license obligations is illegal.
#
# Contact Information:
# ====================
# - mailto:nepi@numurus.com
#

import inspect
import sys


class A:

    def Test(self):
        print("Hello")
        b=B()




class B:
    def __init__(self):
      method_name = self.get_method_name()
      print(method_name)
      caller_method_name = self.test_get_caller_method_name()
      print(caller_method_name)
      caller_class_name = self.get_caller_class_name()
      print(caller_class_name)

    def get_method_name(self):
        method_name = sys._getframe().f_code.co_name
        return method_name

    def test_get_caller_method_name(self):
        return self.get_caller_method_name()

    def get_caller_method_name(self):
        caller_method_name = str(inspect.stack()[1][2])
        return caller_method_name
      
    def get_caller_class_name(self):
        caller_class_name = "None"
        #print(str(inspect.stack()[1][3]))
        frame = inspect.stack()[1][0]
        args, _, _, value_dict = inspect.getargvalues(frame)
        #print(str(args))
        # we check the first parameter for the frame function is
        if len(args) and args[0] == 'self':
          # in that case, 'self' will be referenced in value_dict
          instance = value_dict.get('self', None)
          #print(str(instance))
          if instance:
            caller_class = getattr(instance, '__class__', None)
            #print(str(caller_class))
            caller_class_name = str(caller_class.__name__)
        return caller_class_name

a = A()
a.Test()


  
