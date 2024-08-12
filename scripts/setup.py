from setuptools import setup, find_packages
from codecs import open
from os import path


ext_modules = []

here = path.abspath(path.dirname(__file__))
requires_list = []
try:
    with open(path.join(here, 'requirements.txt'), encoding='utf-8') as f:
        for line in f:
            requires_list.append(str(line))
except FileNotFoundError:
    requires_list = [
        'torch',
        'numpy',
    ]


setup(
    name='pysearch',
    version='0.0',
    packages=find_packages(),
    install_requires=requires_list,
    author='Itamar Mishani',
    author_email='imishani@cs.cmu.edu',
    description='pysearch package',
    url='https://gitlab.com/imishani/search',
)