import re
import sys
import os
from datetime import datetime
import subprocess

os.chdir('.././Middlewares/Third_Party/LwIP/src/apps/http')

subprocess.run('makefsdata_2_2_0.cmd', check=True, shell=True)