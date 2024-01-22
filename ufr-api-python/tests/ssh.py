from lt_api import Link, load
import inspect

factory = load("/home/user/.local/liblt/liblt_ssh.so")

# raspberry = factory.open(1, "user@192.168.0.54")


def raspberry(func):
    code = inspect.getsource(func)
    print(code)
    print(f"{func.__name__}()")

@raspberry
def raspberry_start():
    print("opa")


exit(0)

client = raspberry.open(6, "")
client.write("ls -l")
res = client.read()
client.close()
print(res)

client = raspberry.open(6, "")
client.write("ls -l")
res = client.read()
client.close()
print(res)
