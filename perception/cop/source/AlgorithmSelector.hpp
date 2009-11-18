#include "AlgorithmSelector.h"
#include "XMLTag.h"

template<typename T>
AlgorithmSelector<T>::AlgorithmSelector (XMLTag* node
#ifdef LOGFILE
                                         , LogFile& log) :
    m_logfile(log)
#else /*LOGFILE*/
                                                       )
#endif /*LOGFILE*/
{
  if(node != NULL)
  {
    try
    {
      m_algorithmlist = XMLTag::Load(node, &m_algorithmlist);
    }
    catch(char* pst)
    {
      printf("Error loading Algorithm list: %s\n", pst);
    }
  }
}

//
// Methods
//
template <typename T>
XMLTag* AlgorithmSelector<T>::Save(std::string name )
{
  return XMLTag::Tag(m_algorithmlist, name);
}

template <typename T>
int AlgorithmSelector<T>::InsertInList(AlgorithmEval<T> eval)
{
  m_algorithmlist.push_back(eval);
  return (int)m_algorithmlist.size() - 1;
}

template <typename T>
std::vector<AlgorithmEval<T> > AlgorithmSelector<T>::GetAlgorithmList(std::vector<AlgorithmEval<T> >*)
{
  return m_algorithmlist;
}

template<typename T>
Algorithm<T>* AlgorithmSelector<T>::getAlgorithm(int index)
{
  return m_algorithmlist[index].m_algorithm;
}


template<typename T>
Algorithm<T>* AlgorithmSelector<T>::BestAlgorithm(int type, const Signature &sig, const std::vector<Sensor*> &sensors)
{
  Algorithm<T>* selAlgorithm = NULL;
  double maxEval = -1.0;
  typename std::vector< AlgorithmEval<T> > mv = GetAlgorithmList(&mv);
  printf("TEst\n");
  for(typename std::vector< class AlgorithmEval<T> >::const_iterator iter = mv.begin();
    iter != mv.end(); iter++)
  {
    if(CheckTypeCompatibility((*iter).m_algorithmType, type) && (*iter).m_eval >= maxEval)
    {
      double sig_compatibility = (*iter).m_algorithm->CheckSignature(sig, sensors);
      if(sig_compatibility > 0.0)
      {
        maxEval = (*iter).m_eval * sig_compatibility;
        selAlgorithm = (*iter).m_algorithm;
      }
      else
      {
#ifdef _DEBUG
        printf("The algorithm %s has a signature compatibility of %f (which is too low)\n", (*iter).m_algorithm->GetName().c_str(), sig_compatibility);
#endif
      }
    }
#ifdef _DEBUG
    else
    {
      printf("The algorithm %s is either not compatible with type %d or it is worse than others (%f < %f)\n", (*iter).m_algorithm->GetName().c_str(), type,(*iter).m_eval, maxEval);
    }
#endif
  }
  return selAlgorithm;
}

template<typename T>
void AlgorithmSelector<T>::EvalAlgorithm(Algorithm<T>* alg, double eval, double time, Elem* relatedElem)
{
  typename std::vector<AlgorithmEval<T> > mv = GetAlgorithmList(&mv);
  for(typename std::vector<AlgorithmEval<T> >::iterator iter = mv.begin();
    iter != mv.end(); iter++)
  {
    if((*iter).m_algorithm == alg)
    {
#ifdef _DEBUG
      printf("Add Evaluation to Algorithm %s: %f\n",alg->GetName().c_str(), eval);
#endif
      (*iter).m_eval = ((*iter).m_eval + eval) / 2;    //TODO eval rel situation and object...
      (*iter).m_avgRunTime = ((*iter).m_avgRunTime + time) / 2;
#ifdef LOGFILE
      m_logfile.Log(alg->GetName(), GetName(), time / 100000000 , eval, relatedElem);
#endif /*LOGFILE*/
    }
  }
}

template<typename T>
bool AlgorithmSelector<T>::CheckTypeCompatibility(int listedType, int askedType)
{
  if(askedType == listedType)
    return true;
  if(askedType / 0x100 == listedType / 0x100)
  {
    if(askedType % 0x100 <= listedType % 0x100)
      return true;
  }
  return false;
}

template<typename T>
int AlgorithmSelector<T>::AddAlgorithm(Algorithm<T>* alg, int nType, double dEval, double dTime)
{
  if(dEval <= -1.0)
    dEval = -0.99;
  if(alg == NULL)
    return -1;
  AlgorithmEval<T> pair(alg, nType, dEval, dTime);
  return InsertInList(pair);
}
