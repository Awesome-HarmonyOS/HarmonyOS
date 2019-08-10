from Crypto import Random
from Crypto.PublicKey import RSA

DEFAULT_PRIAVTE_KEY_FILE_NAME = 'private_key.pem'
DEFAULT_PUBLIC_KEY_FILE_NAME = 'public_key.pem'

def main():
	random_generator = Random.new().read
	rsa = RSA.generate(2048, random_generator)
	private_pem = rsa.exportKey()
	
	with open(DEFAULT_PRIAVTE_KEY_FILE_NAME, 'w') as f:
		f.write(private_pem)
		
	public_pem = rsa.publickey().exportKey()
	with open(DEFAULT_PUBLIC_KEY_FILE_NAME, 'w') as f:
		f.write(public_pem)

if __name__ == "__main__":
	main()