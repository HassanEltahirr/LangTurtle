# main.py
from langchain.agents import initialize_agent, AgentType
from tools import teleport_turtle
from llm import get_llm
from prompts import AGENT_PROMPT

llm = get_llm()
tools = [teleport_turtle]
agent = initialize_agent(
    tools, llm, agent=AgentType.ZERO_SHOT_REACT_DESCRIPTION, verbose=True
)

def main():
    while True:
        user_input = input("Command: ")
        response = agent.run(user_input)
        print(response)

if __name__ == "__main__":
    main()