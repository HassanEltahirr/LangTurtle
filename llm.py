# llm.py
from langchain.llms import OpenAI

def get_llm():
    return OpenAI(openai_api_key="")
