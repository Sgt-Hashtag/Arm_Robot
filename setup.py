from setuptools import setup

setup(
    name='adafruit_servokit',
    version='1.0.0',  # You can adjust this based on the version you're using
    description='Adafruit ServoKit Python library',
    author='Adafruit Industries',
    author_email='contact@adafruit.com',
    url='https://github.com/adafruit/Adafruit_Python_ServoKit',  # The URL of the GitHub repo
    packages=['Adafruit_ServoKit'],
    install_requires=[
        'adafruit-blinka',  # Adafruit Blinka is required to use the library
        'RPI.GPIO',         # Required for Raspberry Pi GPIO support (if using Raspberry Pi)
    ],
    classifiers=[
        'Programming Language :: Python :: 3',
        'License :: OSI Approved :: MIT License',
        'Operating System :: POSIX',
        'Operating System :: Microsoft :: Windows',
        'Operating System :: MacOS',
    ],
    python_requires='>=3.6',  # Ensure that the package works with Python 3.6 and later
)

