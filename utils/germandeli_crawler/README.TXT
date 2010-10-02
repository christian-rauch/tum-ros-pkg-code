GermanDeli Website Crawler

===============START=========================
To run the crawler execute the following command:
"python3.1 start_extracting.py absolute_path_to_target_data_dir"

===============ABOUT=========================
The script first extracts the structure of the site in the structure.xml file in the data directory. 
It than follows this file to do the actual extraction of the images. The images are stored in the data 
directory in subfolders, which follow the structure of the site. Additionally some  information 
(like product description, link, id, etc.) is added for every product in the structure.xml file and 
each page is marked as traced, so if the script is interrupted it can continue the crawling, when it is started again.
