import zlib
import os
import sys
import getopt
import  xml.dom.minidom as minidom
#import hashlib
import traceback
from Crypto import Random
from Crypto.Hash import SHA256
from Crypto.Signature import pss as pss
from Crypto.PublicKey import RSA
import binascii
import base64

'''
'''

OK = 0
ERR = 1
STOP = 2
RET_INVALID_ARG = 3
RET_FILE_NOT_EXIST = 4
RET_FILE_PATH_ERR = 5
RET_FILE_OPERATION_ERR = 6
RET_XML_CONFIG_ERR=7


FILE_OPERATE_SIZE = 4 * 1024

TLV_SHA256_CHECKSUM = 1
TLV_CRC_CHECKSUM = TLV_SHA256_CHECKSUM + 1
TLV_SHA256_RSA2048_CHECKSUM = TLV_SHA256_CHECKSUM + 2

INVALID_OFFSET = -1
TLV_T_LEN = 2
TLV_L_LEN = 2

TLV_HEAD_LEN_POS = 4
TLV_TOTAL_LEN_POS = 8
DEFAULT_CONFIG_FILE_NAME = 'config.xml'
DEFAULT_KEY_FILE_NAME = 'private_key.pem'

def min(a, b):
	if a <= b:
		return a
	else:
		return b	

def append_buffer(buffer, pos, value):
	buffer.append(value)
	
def serialize_byte(buffer, value, byte_number = 4, pos = 0, callback = append_buffer):
	max_num = 4
	if byte_number > max_num:
		print 'write byte_number %d err'%(byte_number)
		raise Exception()
		return ERR
	
	mask = 0xff000000
	shift_num = 24
	for i in range(0, max_num - byte_number):
		mask = (mask >> 8)
		shift_num -= 8

	for i in range(0, byte_number):
		callback(buffer, i + pos, ((mask & value) >> shift_num))
		mask = (mask >> 8)
		shift_num -= 8
		
	return OK

class file_writer(object):
	def __init__(self, file_name):
		self.fd = open(file_name, 'w+b')
		
	def __del__(self):
		self.fd.close()
		
	def write(self, value, byte_num = 4, offset = INVALID_OFFSET):
		if offset != INVALID_OFFSET:
			self.fd.seek(offset)	
			
		if isinstance(value, int) or isinstance(value, long):
			buffer = bytearray()
			serialize_byte(buffer, value, byte_num)
		elif isinstance(value, str):
			buffer=bytearray(value)
		else:
			buffer = value
		self.fd.seek(self.fd.tell())
		self.fd.write(buffer)
		
	def read(self, size, offset = INVALID_OFFSET):
		if offset != INVALID_OFFSET:
			self.fd.seek(offset)		
		return self.fd.read(size)
		
	def tell(self):
		return self.fd.tell()
		
	def seek(self, offset):
		self.fd.seek(offset)


	
class config_info(object):
	def __init__(self):
		self.input_file=""
		self.output_file=""
		self.config_file=""	
		self.key_file = ""
		
	def parse_args(self):
		try:
			opts, args = getopt.getopt(sys.argv[1:], 'c:i:o:k')
		except getopt.GetoptError as err:
			print str(err)			
			return RET_INVALID_ARG			
		
		self.config_file = "config.xml"
		for opt, arg in opts:
			if opt == "-c":
				self.config_file = arg
			elif opt == "-i":
				self.input_file = arg
			elif opt == "-o":
				self.output_file = arg
			elif opt == "-k":
				self.key_file = arg
			else:
				pass
		
		if not os.path.isfile(self.input_file):
			print "{0} is not a file!".format(self.input_file)
			return RET_INVALID_ARG
			
		if self.output_file == '':
			self.output_file = os.path.join(os.getcwd(), os.path.basename(self.input_file) + ".out_bin")
		
		if self.config_file == '':
			self.config_file = DEFAULT_CONFIG_FILE_NAME
		
		if self.key_file == '':
			self.key_file = DEFAULT_KEY_FILE_NAME
		
		if not os.path.isfile(self.config_file):
			print "config xml file \"{}\" not exist".format(self.config_file)
			return RET_INVALID_ARG	
		
		return OK
	
	def get_input_file(self):
		return self.input_file
	
	def get_output_file(self):
		return self.output_file
	
	def get_config_file(self):
		return self.config_file
	
			
class checksum(object):
	def name(self):
		return ''
		
	def size(self):
		return 0
	
	def attribute(self):
		return -1
		
	def update(self, buffer):
		pass
	
	def get_checksum(self):
		return ""
		
	def reset(self):
		pass

class sha256_checksum(checksum):
	def __init__(self):
		self.reset()
		
	def name(self):
		return 'sha256'
	
	def attribute(self):
		return TLV_SHA256_CHECKSUM
		
	def size(self):
		return 32
		
	def update(self, buffer):
		self.sha256.update(buffer)
	
	def get_checksum(self):
		print 'sha256 checksum:%s'%(self.sha256.hexdigest())
		return self.sha256.digest()
	
	def reset(self):
		#self.sha256 = hashlib.sha256('')
		self.sha256 = SHA256.new()
		
class sha256_rsa2048_checksum(sha256_checksum):
	def __init__(self, priave_key_file_name):
		sha256_checksum.__init__(self)
		self.priave_key_file_name = priave_key_file_name
		
	def name(self):
		return 'sha256_rsa2048'
	
	def attribute(self):
		return TLV_SHA256_RSA2048_CHECKSUM
		
	def size(self):
		return 256
	
	def get_checksum(self):
		print 'sha256:%s'%(self.sha256.hexdigest())
		with open(self.priave_key_file_name) as f:
			key = f.read()
			rsakey = RSA.importKey(key)
			signer = pss.new(rsakey)
			sign = signer.sign(self.sha256)			
		print 'sha256-rsa2048 checksum:%s'%(binascii.hexlify(sign))
		#print 'base 64:%s' %(base64.b64encode(sign))
		return sign
		
class crc256_checksum(checksum):
	def name(self):
		return 'crc256'
		
class none_checksum(checksum):
	def name(self):
		return 'none'

	
class tlv_type(object):
	def set_writer(self, writer):
		self.writer = writer
		
	def name(self):
		return ''
		
	def get_value(self, value, tlv):
		return [len(value), value]
		
	def write(self, tlv):
		[l, v] = self.get_value(tlv.firstChild.data, tlv)
		if 0 == l:
			return [OK, 0]
		if l < 0:
			return [RET_XML_CONFIG_ERR, 0]
		self.writer.write(v, l)
		return [OK, l]
		
class string_type(tlv_type):
	def name(self):
		return 'string'
		
class integer_type(tlv_type):
	def name(self):
		return 'integer'
		
	def get_value(self, value, tlv):
		value_len = tlv.getAttribute('value_len')
		if value_len == '' or long(value_len, 0) > 4:
			print 'value_len error'
			return [RET_XML_CONFIG_ERR, '']		
		
		return [long(value_len, 0), long(value, 0)]

class software_header(object):	
	def __init__(self, config, writer):
		self.writer = writer
		self.config = config
		self.software_checksum_offset = INVALID_OFFSET
		self.head_len = 0	
		self.software_checksum = ''
		ret = self.__write_config()
		if ret != OK:
			return
		self.__update_head_checksum()

	
	def __write_version(self, dom):
		ver = dom.getElementsByTagName("version")
		version_no = 0		
		if 0 == len(ver):
			print "version tag not exist,using version No.0"
		else:
			version_no = int(ver[0].firstChild.data, 0)
		
		self.writer.write(version_no)
		
	def __write_tlvs(self, dom):
		tlvs = dom.getElementsByTagName("tlv")
		types = [string_type(), integer_type()]
		for type in types:
			type.set_writer(self.writer)
		
		for tlv in tlvs:
			values = tlv.getElementsByTagName("value")
			if len(values) == 0:
				print '%s has no value'%(tlv.nodeName)
				continue
				
			attribute = tlv.getAttribute("attribute")
			if attribute == '':
				print 'attribute empty'
				return RET_XML_CONFIG_ERR
			
			self.writer.write(long(attribute, 0), TLV_T_LEN)
			len_pos = self.writer.tell()
			self.writer.write(0, TLV_L_LEN)	
			values_len = 0
			for value in values:
				ret = ERR
				name = ''
				value_len = 0
				
				for type in types:
					name = value.getAttribute("type")
					if name == type.name():
						[ret, value_len] = type.write(value)
						break
						
				if ret != OK:
					print 'value type %s err'%(name)
					return ret
				values_len += value_len
			pos = self.writer.tell()
			self.writer.write(values_len, TLV_L_LEN, len_pos)
			self.writer.seek(pos)
		return OK
	
	def __init_checksum(self, dom):	
		checksum = dom.getElementsByTagName("checksum")
		if len(checksum) == 0:
			print "no checksum tag in config file"
			return RET_XML_CONFIG_ERR
		algs = [sha256_checksum(), crc256_checksum(), sha256_rsa2048_checksum(self.config.key_file), none_checksum()]
		for alg in algs:
			if alg.name() == checksum[0].firstChild.data:
				self.checksum_alg = alg
				return OK
			
		print 'no checksum alg selected'
		return RET_XML_CONFIG_ERR	
		
				
		
	def __write_config(self):
		dom = minidom.parse(self.config.config_file)
		
		#version No.
		self.__write_version(dom);
		
		#reserve header length
		self.writer.write(0)
		
		#reserve total length
		self.writer.write(0)	

		#tlvs 
		self.__write_tlvs(dom)
		
		ret = self.__init_checksum(dom)
		if ret != OK:
			return ret
		
		size = self.checksum_alg.size()
		if size > 0:			
			self.writer.write(self.checksum_alg.attribute(), TLV_T_LEN)			
			self.writer.write(size, TLV_L_LEN)
			self.software_checksum_offset = self.writer.tell()	
			self.writer.write(bytearray(size))			
		self.head_len = self.writer.tell()
		self.writer.write(self.head_len, 4, TLV_HEAD_LEN_POS)
		self.writer.write(self.head_len + os.path.getsize(self.config.input_file), 4, TLV_TOTAL_LEN_POS)
		self.writer.seek(self.head_len)
		
		return OK
		
	def write_software(self, buffer):
		if self.software_checksum_offset == INVALID_OFFSET:
			return
		self.checksum_alg.update(buffer)
	
	def write_software_end(self):
	#write head and software sha256
		if self.software_checksum_offset == INVALID_OFFSET:
			return
		software_checksum = self.checksum_alg.get_checksum()
		self.writer.write(software_checksum, len(software_checksum), self.software_checksum_offset)
		
	def __update_head_checksum(self):
		read_len = self.head_len
		while read_len > 0:
			tmp_len = min(read_len, FILE_OPERATE_SIZE)
			if read_len == self.head_len:
				buffer = self.writer.read(tmp_len, 0)
			else:				
				buffer = self.writer.read(tmp_len)
			self.checksum_alg.update(buffer)
			read_len -= tmp_len
		
class software_maker(object):
	def __init__(self):
		pass

	def make_software(self, config):
		ret = OK
		input_fd = None
		header = None		
		try:			
			writer = file_writer(config.get_output_file())			
			header = software_header(config, writer)					
			input_fd = open(config.input_file, 'rb')
			while True:
				buffer = input_fd.read(FILE_OPERATE_SIZE)
				if len(buffer) == 0: # EOF or file empty. return hashes	
					header.write_software_end()
					break
				header.write_software(buffer)
				writer.write(buffer)
				
		except IOError as e:
			print "IOError {0}".format(str(e))	
			ret = RET_FILE_OPERATION_ERR
			traceback.print_exc()
			
		except  Exception as err:
			print 'except hanppen!' + str(err)
			ret = RET_FILE_OPERATION_ERR
			traceback.print_exc()

		finally:
			if not input_fd is None:
				input_fd.close()			

		print 'make software %s to %s ,length %d, ret %d'%(config.input_file, config.output_file, os.path.getsize(config.input_file), ret)
		
		return ret

def print_usage():
	print "Usage: {} [-c config_xml_file]  [-o output_file] -i input_file [-k key_file]".format(sys.argv[0])
	

	
def main():
	config = config_info()	
	ret = config.parse_args()	
	if ret != OK:
		print "parse args err ret %d"%(ret)
		print_usage()
		return ret
	
	maker = software_maker()
	ret = maker.make_software(config)
	if ret != OK:
		print_usage()
	return ret
	
if __name__ == "__main__":
	main()
	
