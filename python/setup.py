"""Install the Python USB2CAN and Lingzu motor interfaces."""
from setuptools import setup

setup(name="humanoid-usb2can", version="1.0.1",
      description="SOULDE USB2CAN serial transport and Lingzu motor codecs",
      packages=["lingzu", "pyusb2can"],
      install_requires=["pyserial>=3.5,<4"], python_requires=">=3.8",
      extras_require={"can": ["python-can>=4.3,<5"]}, license="Apache-2.0")
