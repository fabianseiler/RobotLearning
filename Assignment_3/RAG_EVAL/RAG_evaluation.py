"""
Simple RAG evaluation focused on accuracy analysis with confidence scoring
Binary scoring: 1 for correct, 0 for incorrect
Includes confidence scores using structured outputs
"""

from langchain_community.document_loaders import WebBaseLoader
from langchain_text_splitters import RecursiveCharacterTextSplitter
from langchain_chroma import Chroma
from langchain_ollama import OllamaEmbeddings, ChatOllama
from langchain_core.output_parsers import StrOutputParser
from langchain_core.prompts import ChatPromptTemplate
from langchain_core.runnables import RunnablePassthrough
from pydantic import BaseModel, Field
import time
import json
from datetime import datetime
from typing import List, Optional


class ConfidentResponse(BaseModel):
    """Structured response with confidence score"""
    answer: str = Field(description="The answer to the question (max 3 words)")
    confidence: float = Field(description="Confidence score between 0.0 and 1.0", ge=0.0, le=1.0)
    reasoning: Optional[str] = Field(description="Brief explanation of the reasoning", default="")


class AccuracyAnalyzer:
    def __init__(self):
        # Ground truth answers for accuracy evaluation
        self.ground_truth = {
            "Who is the current Pope?": [
                "Pope Leo XIV", "Leo XIV", "Robert Francis Cardinal Prevost"
            ],
            "Who is the current president of the USA?": [
                "Donald Trump", "President Donald Trump", "Trump"
            ],
            "What is the latest iPhone model released?": [
                "iPhone 16", "Apple iPhone 16", "iPhone 16 Pro"
            ],
            "Who won the UEFA Champions League title in 2025?": [
                "Paris Saint-Germain", "PSG"
            ],
            "Who is the current president of Switzerland?": [
                "Karin Keller-Sutter", "Keller-Sutter"
            ],
            "Which two countries joined the Schengen Area?": [
                "Romania and Bulgaria", "Bulgaria and Romania", "Romania, Bulgaria"
            ],
            "Who became the governor of Vatican City?": [
                "Sister Raffaella Petrini", "Raffaella Petrini", "Sr. Raffaella Petrini"
            ],
            "Which airline became the first to resume flights to Damascus, Syria in January 2025?": [
                "Qatar Airways", "QR", "Qatar Airways Group"
            ],
            "Which major California wildfires were contained in early 2025?": [
                "Palisades Fire and Eaton Fire", "Eaton Fire and Palisades Fire", "Palisades and Eaton fires"
            ],
            "Which U.S. company's buyout by Nippon Steel was blocked by President Biden in January 2025?": [
                "U.S. Steel", "United States Steel Corporation", "USS"
            ]
        }

    def evaluate_accuracy(self, question, response):
        """Evaluate response accuracy with binary scoring (1 or 0)"""
        if not response or question not in self.ground_truth:
            return {
                "score": 0,
                "explanation": "No response or no ground truth available"
            }

        correct_answers = self.ground_truth[question]
        response_lower = response.lower()

        # Check if any correct answer is found in the response
        for correct_answer in correct_answers:
            if correct_answer.lower() in response_lower:
                return {
                    "score": 1,
                    "explanation": f"Found correct answer: '{correct_answer}'"
                }

        return {
            "score": 0,
            "explanation": "No correct answers found in response"
        }


def setup_knowledge_base():
    """Set up RAG knowledge base with reliable sources"""
    print("Setting up knowledge base...")

    urls = [
        "https://en.wikipedia.org/wiki/Pope_Leo_XIV",  # Pope Leo XIV (Robert Francis Prevost)
        "https://en.wikipedia.org/wiki/Donald_Trump",  # President Trump (confirmed current)
        "https://en.wikipedia.org/wiki/IPhone_16",  # iPhone 16 (latest model, with iPhone 16e variant)
        "https://en.wikipedia.org/wiki/2025_UEFA_Champions_League_Final",  # PSG winning first UCL
        "https://en.wikipedia.org/wiki/President_of_the_Swiss_Confederation",  # Swiss President 2025
        "https://home-affairs.ec.europa.eu/news/bulgaria-and-romania-join-schengen-area-2025-01-03_en", # Romania Bulgaria Schengen
        "https://apnews.com/article/pope-vatican-women-petrini-89e1e1af3ea355a7d6bb06148814364b",    # Sister Raffaella Petrini Vatican governor
        "https://aviationweek.com/air-transport/airports-networks/qatar-airways-resume-syria-flights",     # Qatar Airways Damascus flights
        "https://en.wikipedia.org/wiki/2025_in_the_United_States",  # Palisades and Eaton fires contained
        "https://www.nytimes.com/2025/01/03/us/politics/us-steel-nippon-biden.html",     # Biden blocks Nippon Steel U.S. Steel buyout
    ]

    all_documents = []
    successful_loads = 0

    for url in urls:
        try:
            loader = WebBaseLoader(url)
            docs = loader.load()
            all_documents.extend(docs)
            successful_loads += 1
            print(f"✓ Loaded: {url}")
        except Exception as e:
            print(f"✗ Failed: {url} - {e}")

    # Fallback sources if main sources fail
    if not all_documents:
        print("Using fallback sources...")
        fallback_urls = [
            "https://en.wikipedia.org/wiki/Pope",
            "https://en.wikipedia.org/wiki/NATO",
            "https://en.wikipedia.org/wiki/List_of_chemical_elements"
        ]

        for url in fallback_urls:
            try:
                loader = WebBaseLoader(url)
                docs = loader.load()
                all_documents.extend(docs)
                print(f"✓ Fallback loaded: {url}")
            except Exception as e:
                print(f"✗ Fallback failed: {url} - {e}")

    # Split documents into chunks
    text_splitter = RecursiveCharacterTextSplitter(chunk_size=500, chunk_overlap=50)
    all_splits = text_splitter.split_documents(all_documents)

    # Create vector store
    embeddings = OllamaEmbeddings(model="nomic-embed-text")
    vectorstore = Chroma.from_documents(documents=all_splits, embedding=embeddings)

    print(f"Knowledge base ready: {len(all_splits)} chunks from {successful_loads} sources\n")
    return vectorstore


def format_docs(docs):
    """Format retrieved documents for context"""
    return "\n\n".join(doc.page_content for doc in docs)


def evaluate_model(model_name, questions, vectorstore, analyzer):
    """Evaluate both base and RAG models for accuracy with confidence scoring"""
    print(f"Evaluating {model_name}")
    print("=" * 50)

    # Initialize models with structured output
    base_model = ChatOllama(model=model_name, format="json")
    rag_model = ChatOllama(model=model_name, format="json")

    # Base model template with confidence scoring
    base_template = """
    Answer the following question with maximum 3 words and provide a confidence score.

    Question: {question}

    You must respond in JSON format with the following structure:
    {{
        "answer": "your answer (max 3 words)" and if you dont know just write I dont know.,
        "confidence": the value of the confidence score between 0 and 1, by 1 meaning 100% sure and 0 for not sure, if you dont know just write 0,
        "reasoning": "brief explanation"
    }}

    IMPORTANT: Be honest about your confidence level. Consider these guidelines:
    - Use 0.9-1.0 only for basic facts you're absolutely certain about
    - Use 0.7-0.8 for information you're quite confident about but could have some uncertainty
    - Use 0.4-0.6 for educated guesses or when you have partial information
    - Use 0.1-0.3 for wild guesses or highly uncertain responses
    - Use 0.0 when you truly don't know

    Do NOT default to high confidence scores. Vary your confidence based on actual certainty.
    """

    # RAG template with confidence scoring
    rag_template = """
    You are an assistant for question-answering tasks. Use the following pieces of retrieved context 
    to answer the question. If you don't know the answer, just say that you don't know. 
    Provide your answer in maximum 3 words and include a confidence score.

    <context>
    {context}
    </context>

    Question: {question}

    You must respond in JSON format with the following structure:
    {{
        "answer": "your answer (max 3 words)" and if you dont know just write I dont know.,
        "confidence": the value of the confidence score between 0 and 1, by 1 meaning 100% sure and 0 absolutely not sure, if you dont know just write 0,
        "reasoning": "brief explanation based on context"
    }}

    IMPORTANT: Base your confidence on how well the context supports your answer:
    - Use 0.9-1.0 when the context explicitly and clearly states the answer
    - Use 0.7-0.8 when the context strongly implies the answer but isn't completely explicit
    - Use 0.4-0.6 when the context provides partial or indirect support for your answer
    - Use 0.1-0.3 when you're making an educated guess based on weak contextual clues
    - Use 0.0 when the context doesn't support an answer at all

    Do NOT default to high confidence. Your confidence must reflect how well the context supports your specific answer.
    """

    base_prompt = ChatPromptTemplate.from_template(base_template)
    rag_prompt = ChatPromptTemplate.from_template(rag_template)
    retriever = vectorstore.as_retriever(search_kwargs={"k": 4})

    # Create chains
    base_chain = base_prompt | base_model | StrOutputParser()
    rag_chain = (
            {"context": retriever | format_docs, "question": RunnablePassthrough()}
            | rag_prompt
            | rag_model
            | StrOutputParser()
    )

    results = {}

    for i, question in enumerate(questions, 1):
        print(f"\n[{i}/{len(questions)}] {question}")
        print("-" * 40)

        # Evaluate base model
        print("BASE MODEL:")
        try:
            base_response_raw = base_chain.invoke({"question": question})
            base_response_json = json.loads(base_response_raw)

            base_answer = base_response_json.get("answer", "")
            base_confidence = base_response_json.get("confidence", 0.0)
            base_reasoning = base_response_json.get("reasoning", "")

            base_accuracy = analyzer.evaluate_accuracy(question, base_answer)

            print(f"  Response: {base_answer}")
            print(f"  Confidence: {base_confidence:.2f}")
            print(f"  Reasoning: {base_reasoning}")
            print(f"  Accuracy: {base_accuracy['score']} - {base_accuracy['explanation']}")

            base_result = {
                "response": base_answer,
                "confidence": base_confidence,
                "reasoning": base_reasoning,
                "score": base_accuracy['score'],
                "explanation": base_accuracy['explanation'],
                "raw_response": base_response_raw
            }
        except Exception as e:
            base_result = {
                "response": "",
                "confidence": 0.0,
                "reasoning": "",
                "score": 0,
                "error": str(e)
            }
            print(f"  Error: {e}")

        # Evaluate RAG model
        print("\nRAG MODEL:")
        try:
            rag_response_raw = rag_chain.invoke(question)
            rag_response_json = json.loads(rag_response_raw)

            rag_answer = rag_response_json.get("answer", "")
            rag_confidence = rag_response_json.get("confidence", 0.0)
            rag_reasoning = rag_response_json.get("reasoning", "")

            rag_accuracy = analyzer.evaluate_accuracy(question, rag_answer)
            retrieved_docs = vectorstore.similarity_search(question, k=4)

            print(f"  Retrieved docs: {len(retrieved_docs)}")
            print(f"  Response: {rag_answer}")
            print(f"  Confidence: {rag_confidence:.2f}")
            print(f"  Reasoning: {rag_reasoning}")
            print(f"  Accuracy: {rag_accuracy['score']} - {rag_accuracy['explanation']}")

            rag_result = {
                "response": rag_answer,
                "confidence": rag_confidence,
                "reasoning": rag_reasoning,
                "score": rag_accuracy['score'],
                "explanation": rag_accuracy['explanation'],
                "retrieved_docs_count": len(retrieved_docs),
                "context_preview": format_docs(retrieved_docs)[:500] + "..." if len(
                    format_docs(retrieved_docs)) > 500 else format_docs(retrieved_docs),
                "raw_response": rag_response_raw
            }
        except Exception as e:
            rag_result = {
                "response": "",
                "confidence": 0.0,
                "reasoning": "",
                "score": 0,
                "error": str(e),
                "retrieved_docs_count": 0,
                "context_preview": ""
            }
            print(f"  Error: {e}")

        # Compare results
        print("\nCOMPARISON:")
        if "error" not in base_result and "error" not in rag_result:
            accuracy_improvement = rag_result['score'] - base_result['score']
            confidence_change = rag_result['confidence'] - base_result['confidence']

            if accuracy_improvement > 0:
                print(f"  ✓ RAG improved accuracy (+{accuracy_improvement})")
            elif accuracy_improvement < 0:
                print(f"  ✗ RAG decreased accuracy ({accuracy_improvement})")
            else:
                print(f"  → No change in accuracy")

            print(f"  Confidence change: {confidence_change:+.2f}")
            print(f"  Base confidence: {base_result['confidence']:.2f}")
            print(f"  RAG confidence: {rag_result['confidence']:.2f}")

        results[question] = {
            "base": base_result,
            "rag": rag_result,
            "ground_truth": analyzer.ground_truth.get(question, [])
        }

        time.sleep(1)  # Rate limiting

    return results


def generate_report(all_results):
    """Generate accuracy analysis report with confidence metrics"""
    print(f"\n{'=' * 60}")
    print("ACCURACY ANALYSIS REPORT WITH CONFIDENCE SCORING")
    print(f"{'=' * 60}")

    report = {
        "timestamp": datetime.now().isoformat(),
        "model_performance": {},
        "detailed_results": all_results
    }

    for model_name, model_results in all_results.items():
        print(f"\n{model_name.upper()}:")
        print("-" * 30)

        base_scores = []
        rag_scores = []
        base_confidences = []
        rag_confidences = []
        improvements = 0
        total_questions = 0

        for question, results in model_results.items():
            if "error" not in results["base"] and "error" not in results["rag"]:
                total_questions += 1
                base_score = results["base"]["score"]
                rag_score = results["rag"]["score"]
                base_confidence = results["base"]["confidence"]
                rag_confidence = results["rag"]["confidence"]

                base_scores.append(base_score)
                rag_scores.append(rag_score)
                base_confidences.append(base_confidence)
                rag_confidences.append(rag_confidence)

                if rag_score > base_score:
                    improvements += 1

        if total_questions > 0:
            base_accuracy = sum(base_scores) / len(base_scores)
            rag_accuracy = sum(rag_scores) / len(rag_scores)
            base_avg_confidence = sum(base_confidences) / len(base_confidences)
            rag_avg_confidence = sum(rag_confidences) / len(rag_confidences)
            improvement_rate = improvements / total_questions

            print(f"Base Model Accuracy:      {base_accuracy:.2%} ({sum(base_scores)}/{len(base_scores)})")
            print(f"RAG Model Accuracy:       {rag_accuracy:.2%} ({sum(rag_scores)}/{len(rag_scores)})")
            print(f"Accuracy Improvement:     {rag_accuracy - base_accuracy:+.2%}")
            print(f"Questions Improved:       {improvements}/{total_questions} ({improvement_rate:.1%})")
            print(f"Base Avg Confidence:      {base_avg_confidence:.2f}")
            print(f"RAG Avg Confidence:       {rag_avg_confidence:.2f}")
            print(f"Confidence Change:        {rag_avg_confidence - base_avg_confidence:+.2f}")

            # Store in report
            report["model_performance"][model_name] = {
                "base_accuracy": base_accuracy,
                "rag_accuracy": rag_accuracy,
                "improvement": rag_accuracy - base_accuracy,
                "improvement_rate": improvement_rate,
                "base_avg_confidence": base_avg_confidence,
                "rag_avg_confidence": rag_avg_confidence,
                "confidence_change": rag_avg_confidence - base_avg_confidence,
                "total_questions": total_questions,
                "individual_confidences": {
                    "base": base_confidences,
                    "rag": rag_confidences
                }
            }

    # Save report
    filename = f"accuracy_confidence_report_{datetime.now().strftime('%Y%m%d_%H%M%S')}.json"
    with open(filename, 'w') as f:
        json.dump(report, f, indent=2)

    print(f"\nReport saved to: {filename}")
    return report


def main():
    """Main evaluation function"""
    print("RAG ACCURACY EVALUATION WITH CONFIDENCE SCORING")
    print("=" * 50)

    # Initialize components
    analyzer = AccuracyAnalyzer()
    vectorstore = setup_knowledge_base()

    # Test questions
    questions = [
        "Who is the current Pope?",
        "Who is the current president of the USA?",
        "What is the latest iPhone model released?",
        "Who won the UEFA Champions League title in 2025?",
        "Who is the current president of Switzerland?",
        "Which two countries joined the Schengen Area?",
        "Who became the governor of Vatican City?",
        "Which airline became the first to resume flights to Damascus, Syria in January 2025?",
        "Which major California wildfires were contained in early 2025?",
        "Which U.S. company's buyout by Nippon Steel was blocked by President Biden in January 2025?",
    ]

    # Models to evaluate
    models = ["gemma3:27b"]

    # Run evaluations
    all_results = {}
    for model_name in models:
        all_results[model_name] = evaluate_model(model_name, questions, vectorstore, analyzer)

    generate_report(all_results)
    print("\nEvaluation complete!")


if __name__ == "__main__":
    main()
