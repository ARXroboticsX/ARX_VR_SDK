from distutils.core import setup
from Cython.Build import cythonize

setup(ext_modules=cythonize(["tcp_sender.py",
                             "communication.py", "service.py", "thread_pauser.py",
                             "publisher.py", "unity_service.py", "exceptions.py", "subscriber.py"]))
