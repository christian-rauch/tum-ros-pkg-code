package edu.tum.cs.ias.knowrob.utils;

import java.io.*;


/**
 * 
 * Utilities related to the ROS system
 * 
 * @author Moritz Tenorth, tenorth@cs.tum.edu
 *
 */


public class ROSUtils {

	
	/**
	 * Finds a ROS package using rospack and returns its path. 
	 * @param name of the ROS package
	 * @return path to the package - if it was found<br>
	 * <tt>null</tt> - otherwise
	 */
	public static String rospackFind(String pkg) {

		String path = null;
		try
		{
			Process p = Runtime.getRuntime().exec("rospack find " + pkg);
			p.waitFor();
			BufferedReader br = new BufferedReader(new InputStreamReader(p.getInputStream()));

			if ((path = br.readLine()) != null){
				;//System.out.println("Package: " + pkg + ", Path: " + path);                    
			} 
			else
				;//System.out.println("Package: " + pkg + ", Error: package not found!");
		}
		catch (Exception e)
		{
			e.printStackTrace();
		}
		return path;
	}
	

}
