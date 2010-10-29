package edu.tum.cs.ias.knowrob.json_prolog;

import java.util.Hashtable;

import jpl.Term;

public final class PrologAllSolutions implements PrologSolutions {

  private int currentIndex = 0;
  Hashtable<String, jpl.Term>[] solutions;
  
  @SuppressWarnings("unchecked")
  PrologAllSolutions(jpl.Query query) {
    solutions = (Hashtable<String, jpl.Term>[])query.allSolutions();
  }
  
  @Override
  public void close() {
    // This method doesn't need to do anything here. We already closed the query. 
  }  
  
  @Override
  public void reset() {
    currentIndex = 0;
  }

  @Override
  public Hashtable<String, Term> nextSolution() {
    Hashtable<String, jpl.Term> result = solutions[currentIndex];
    currentIndex++;
    return result;
  }

  @Override
  public boolean hasMoreSolutions() {
    return currentIndex < solutions.length;
  }

}
