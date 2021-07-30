import sys, os

if __name__ == '__main__':
    if str(sys.argv[1]) == 'install':
        os.system('sudo ./pico_setup.sh')
