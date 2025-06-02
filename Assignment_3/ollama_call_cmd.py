import os
import json
import time
import requests


def ask_llm(query, model):
    """
    Send a query to an Ollama model using its REST API.
    """
    url = "http://localhost:11434/api/generate"
    payload = {
        "model": model,
        "prompt": query,
        "stream": False
    }
    try:
        response = requests.post(url, json=payload)
        response.raise_for_status()
        return response.json()["response"].strip()
    except requests.RequestException as e:
        return f"Error: {e}"


if __name__ == "__main__":

    def evaluate_dataset(path="./Reasoning_Tasks/problem_dataset.json",
                         save_path="./Reasoning_Tasks/results.json"):
        """
        Evaluates the reasoning dataset for three gemma models
        """
        results = {}

        for model in ["gemma3:1b", "gemma3:4b", "gemma3:27b"]:

            # Load the JSON file
            with open(path, "r") as f:
                dataset = json.load(f)

            for config in dataset:

                problem = config["problem"]
                prompt = f"You need to think and respond. Put your thinking in <think></think>. Put your solution in <SOLUTION></SOLUTION>. {problem}"

                print(f"Querying {model} for task {config['ID']}...")
                answer = ask_llm(prompt, model)

                # Update results with the model's answer
                results[f"{config['ID']}_{model}"] = answer

                # Optional: short pause between requests
                time.sleep(1)

        # Save updated config
        with open(save_path, "w") as f:
            json.dump(results, f)


    def analyze_dataset():

        with open("./Reasoning_Tasks/problem_dataset.json", "r") as f:
            dataset = json.load(f)

        with open("./Reasoning_Tasks/results.json", "r") as f:
            results = json.load(f)

        for result in results:
            print(f"{result}")
            print(f"{results[result]}")
            print(f"Correct answer: {dataset[int(result.split('_')[0])-1]['correct_answer']}\n\n\n")


    #evaluate_dataset()
    evaluate_dataset(path="./Reasoning_Tasks/problem_strawberry.json",save_path="./Reasoning_Tasks/results_strawberry.json")
    #analyze_dataset()

