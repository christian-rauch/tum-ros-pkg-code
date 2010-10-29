package edu.tum.cs.ias.knowrob.json_prolog;

import java.util.Hashtable;

import jpl.Term;

public class PrologIncrementalSolutions implements PrologSolutions {
  private jpl.Query query;

  public PrologIncrementalSolutions(jpl.Query query) {
    this.query = query;
  }
  
  @Override
  public void close() {
    query.rewind();
  }

  @SuppressWarnings("unchecked")
  @Override
  public Hashtable<String, Term> nextSolution() {
    return (Hashtable<String, Term>) query.nextElement();
  }

  @Override
  public void reset() {
    query.rewind();
  }

  @Override
  public boolean hasMoreSolutions() {
    return query.hasMoreElements();
  }

}
