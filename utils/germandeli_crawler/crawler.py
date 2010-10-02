import urllib
import xml.dom.minidom
import re
import os
import logging
import sys
import time
from xml.dom.minidom import Document
from threading import Thread
import threading



###################  Some helper functions and classes ##########################


# Downloads the content of a web page
def get_page_content(page):
    url = urllib.urlopen(page)
    raw_content = url.read()
    url.close()
    return raw_content.decode('iso-8859-1', 'ignore')
    
# Checks if a directory exists and if not creates it
def ensure_directory(dir):
    if not os.path.exists(dir):
        os.makedirs(dir)
        
# Removes the extention of a given link. For example strip_extention("test.htm") == "test"
def get_id_from_url(page):
    return page[page.rfind('/')+1 : page.rfind('.')]



###################  A class for the crawler threads  ##########################

class ProcessorThread(Thread):
    def __init__ (self, crawler, item, page, data_directory, xml_node):
        Thread.__init__(self)
        self.crawler = crawler
        self.item = item
        self.page = page
        self.data_directory = data_directory
        self.xml_node = xml_node
        
    def run(self):
        product = self.crawler.process_product_page(self.page)
        self.crawler.save_product(self.item, product['image_url'], self.data_directory)
        
        product_element = self.crawler.doc.createElement('product')
        product_element.setAttribute('description', product['image_description'])
        product_element.setAttribute('perishability', product['perishability'])
        product_element.setAttribute('weight', product['info_weight'])
        product_element.setAttribute('brand', product['info_brand'])
        product_element.setAttribute('country_of_origin', product['info_countryoforigin'])
        product_element.setAttribute('sale_price', product['info_sale_price'])
        product_element.setAttribute('product_code', product['info_code'])
        product_element.setAttribute('link', self.page)
        product_element.setAttribute('id', self.item)
        product_element.setAttribute('location', self.data_directory)
        self.xml_node.appendChild(product_element)    
        
    
    
    
###################  The main crawler class  ##########################
      
class GermanDeliCrawler:

    base_url = "http://www.germandeli.com/"
    data_directory = ""
    
    doc = None
    logger = None

    def __init__(self, data_directory, max_threads = 999999, base_url = "http://www.germandeli.com/"):
        self.base_url = base_url      
        self.max_threads = max_threads  
        self.data_directory = data_directory
        self.doc = Document()
        root = self.doc.createElement('group')
        self.doc.appendChild(root)
        
        self.logger = logging.getLogger('GermanDeliCrawer')
        file_handler = logging.FileHandler(os.path.join(self.data_directory, 'crawler.log'))
        console_handler = logging.StreamHandler(sys.stdout)
        
        formatter = logging.Formatter('%(asctime)s %(levelname)s %(message)s')
        file_handler.setFormatter(formatter)
        console_handler.setFormatter(formatter)
        self.logger.addHandler(file_handler)
        self.logger.addHandler(console_handler)
        self.logger.setLevel(logging.INFO)
        
    
    
    # Extracts the structure of the Germandeli.com site and stores it in a XML file
    def extract_germandeli_structure(self):
        self.log("Extracting site structure...");
        url = urllib.urlopen('http://www.germandeli.com')
        content = url.read()
        url.close()
            
        nav_start = '<ul id="nav">'.encode();
        nav_end = '\n</div>\n<div id="bodyshell" class="clear">'.encode()
            
        nav_start_id = content.find(nav_start)
        nav_end_id = content.find(nav_end)
        
        nav_string = content[nav_start_id : nav_end_id].decode('iso-8859-1', 'ignore').replace("&", "&amp;").replace("<BR>","")
        
        structure = xml.dom.minidom.parseString(nav_string.encode("utf-8"))
        
        self.trace_structure_node_to_xml(structure.firstChild, self.doc.firstChild)
        
        self.save_document()
    
    # Helper function for the extraction of the structure of the site
    def trace_structure_node_to_xml(self, node, xml_node):
        if node.tagName == 'a':
            if node.firstChild.nodeValue is not None:
                xml_node.setAttribute('name', node.firstChild.nodeValue.encode('iso-8859-1').decode('utf8', 'ignore'))
                xml_node.setAttribute('link', node.getAttribute('href'))
            return
        
        if node.tagName == 'li':
            new_node = self.doc.createElement('page')
            xml_node.appendChild(new_node)
            xml_node.tagName = 'group'
            xml_node = new_node
            
        for child in node.childNodes:
            self.trace_structure_node_to_xml(child, xml_node)
    
    # Loads the structure of the Germandeli.com site from a XML file
    def load_structure_from_file(self):
        self.log("Loading site structure from file...");
        self.doc = xml.dom.minidom.parse(os.path.join(self.data_directory,'structure.xml')) 
        
    # Saves the structure XML file
    def save_document(self):
        f = open(os.path.join(self.data_directory, 'structure.xml'), 'wb')
        f.write(self.doc.toprettyxml('\t', '\n', 'utf-8'))
        f.close()
        
    
    
    
    
    
    
    # processes a node form the XML description of the site structure
    def process_node(self, node, data_directory):
        if node.nodeType != node.ELEMENT_NODE:
            return
        
        # if the node is of type page we should get all products listed there
        if node.tagName == 'page':
            page_directory = os.path.join(data_directory, get_id_from_url(node.getAttribute('link')))
            ensure_directory(page_directory)
            self.process_page(node.getAttribute('link'), node, page_directory)
            
        # if the node is of type groups, we should create a new directory and process the children of the groups
        elif node.tagName == 'group':
            group_directory = os.path.join(data_directory, get_id_from_url(node.getAttribute('link')))
            ensure_directory(group_directory)
            for child in node.childNodes:
                self.process_node(child, group_directory)
                
        # if the node is unknown just process its children
        else:
            for child in node.childNodes:
                self.process_node(child, data_directory)

        
    
    # processes a page with products
    def process_page(self, page, xml_node, data_directory):
        if xml_node.getAttribute('traced') == '1':
            self.log("Skipping category:   " + page)
            return;
        
        self.log("Processing category:   " + page)
        
        content = get_page_content(self.base_url + page)
        head_content = content[1:content.find('<body')]
    
        items = re.findall('window\.item[0-9].+?pagingItem\(\"(.+?)\"', head_content)
       
        item_number = 0;
        item_count = len(items)
        for item in items:
            item_number = item_number + 1
            
            page = self.base_url + item + ".html"
            self.log("\tProcessing product (%d/%d):   %s"  % (item_number, item_count, page))  
            
            # start a new thread, which should download the picture and description for a single product
            thread = ProcessorThread(self, item, page, data_directory, xml_node)
            thread.start()
            
            if item_number % self.max_threads == 0:
                self.wait_for_threads()
            
            
        # Wait for all threads to finish
        self.wait_for_threads()
            
        # mark the node as traced
        xml_node.setAttribute('traced', '1')
        self.save_document()
        
    def wait_for_threads(self):
        sys.stdout.write(str(threading.active_count()))
        while threading.active_count() > 1:
            time.sleep(0.5)
            sys.stdout.write(', ' + str(threading.active_count()))
            sys.stdout.flush()
            
        sys.stdout.write('\n');
        
       
    # Processes a single product
    def process_product_page(self, product_page):
        content = get_page_content(product_page)
        
        result = {};
        
        # image information
        image_div_start_string = '<div class="item-images">'
        
        image_div_start = content.find(image_div_start_string)
        image_div_end = content.find('</div>', image_div_start)
        
        image_content = content[image_div_start : image_div_end]
        
        m = re.search('<img.*?src="(.*?)".*?alt="(.*?)"', image_content)

        result['image_url'] = m.group(1);
        result['image_description'] = m.group(2)
        
        
        
        # perishability
        m = re.search('<div.*?class="perishable.*?>.*?<img.*?alt="(.*?)".*?>', content);
        result['perishability'] = m.group(1) if m else ''
     
        
        
        
        # info table
        m = re.search('<table.*?id="product-info-table".*?>(.*?)</table>', content);
        info_content = m.group(1) if m else ''
        
        # look for brand
        m = re.search('<tr.*?class="brand".*?>.*?<td>(.*?)</td>', info_content)
        result['info_brand'] =  m.group(1) if m else ''
        
        # look for countryoforigin
        m = re.search('<tr.*?class="countryoforigin".*?>.*?<td>(.*?)</td>', info_content)
        result['info_countryoforigin'] = m.group(1) if m else ''
        
        # look for code
        m = re.search('<tr.*?class="code".*?>.*?<td>(.*?)</td>', info_content)
        result['info_code'] = m.group(1) if m else ''
        
        # look for weight
        m = re.search('<tr.*?class="weight".*?>.*?<td>(.*?)</td>', info_content)
        result['info_weight'] = m.group(1) if m else ''
        
        # look for sale-price
        m = re.search('<tr.*?class="sale-price".*?>.*?<td>(.*?)</td>', info_content)
        result['info_sale_price'] = m.group(1) if m else ''
        
        return result        
    
    
    # Saves the product information (currently just the picture)
    def save_product(self, product_id, picture_url, data_directory):
        urllib.urlretrieve(picture_url, os.path.join(data_directory, product_id + ".jpg"))
    





    def start(self):
        self.process_node(self.doc.firstChild, self.data_directory)
        
    def log(self, message):
        self.logger.info(message)
        
        
        
        
        
        
        
        
    
    
