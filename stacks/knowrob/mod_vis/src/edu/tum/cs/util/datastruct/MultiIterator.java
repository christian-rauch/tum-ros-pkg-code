/*
 * Created on Sep 22, 2009
 *
 * TODO To change the template for this generated file go to
 * Window - Preferences - Java - Code Style - Code Templates
 */
package edu.tum.cs.util.datastruct;

import java.util.Iterator;
import java.util.Vector;

public class MultiIterator<T> implements Iterable<T> {

	Vector<Iterable<T>> collections = new Vector<Iterable<T>>();	
	
	public Iterator<T> iterator() {
		return new LocalIterator(collections);
	}
	
	public void add(Iterable<T> i) {
		collections.add(i);
	}
	
	public void add(T obj) {
		Vector<T> v = new Vector<T>(1);
		v.add(obj);
		add(v);
	}
	
	public boolean isEmpty() {
		if(collections.size() == 0)
			return true;
		return iterator().hasNext();
	}
	
	public class LocalIterator implements Iterator<T> {

		Vector<Iterable<T>> collections;
		Iterator<Iterable<T>> ii;
		Iterator<T> curI;
		
		public LocalIterator(Iterable<Iterable<T>> collections) {
			ii = collections.iterator();
			curI = ii.next().iterator();
		}
		
		public boolean hasNext() {		
			if(curI.hasNext())
				return true;
			while(!curI.hasNext()) {
				if(!ii.hasNext())
					return false;
				curI = ii.next().iterator();
			}
			return true;
		}

		public T next() {
			return curI.next();
		}

		public void remove() {
			throw new RuntimeException("not implemented");
		}		
	}
	
}
