import urllib2
import time

#this script is used to check if the network is ready
def internet_on():
	while True:
		try:
			urllib2.urlopen('http://216.58.192.142', timeout=1)
			return True
		except urllib2.URLError as err:
			return False
