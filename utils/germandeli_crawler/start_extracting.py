from crawler import GermanDeliCrawler
import os
import sys


if __name__ == '__main__':

	if len(sys.argv) < 2:
		print('Data directory not specified! Usage start_extracting.py <data_directory>')
		sys.exit(1)

	data_directory = sys.argv[1]
	
	# instantiate the crawler with default parameters
	crawler = GermanDeliCrawler(data_directory)
	
	
	# if the file structure.xml exists load it and proceed with the crawling
	if os.path.exists(os.path.join(crawler.data_directory, 'structure.xml')):
		crawler.load_structure_from_file()
		
	# if not extract the structure of the site again and create the structure.xml file
	else:
		crawler.extract_germandeli_structure();
	
	#print(crawler.doc.toprettyxml('   ', '\n', 'utf-8').decode())
	
	# start crawling
	crawler.start()

