package edu.tum.cs.ias.knowrob.json_prolog;

import java.util.Hashtable;

public interface PrologSolutions {
  boolean hasMoreSolutions();
  Hashtable<String, jpl.Term> nextSolution();
  void close();
  void reset();
}
