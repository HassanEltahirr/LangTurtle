from langchain.agents import initialize_agent, AgentType
from tools import teleport_turtle, return_to_start
from llm import get_llm
from prompts import AGENT_PROMPT
from tools import teleport_turtle, return_to_start, thank_you_turtle_tool

llm = get_llm()
tools = [teleport_turtle, return_to_start, thank_you_turtle_tool]

agent = initialize_agent(
    tools,
    llm,
    agent=AgentType.ZERO_SHOT_REACT_DESCRIPTION,
    verbose=True,
    agent_kwargs={"prefix": AGENT_PROMPT}
)

def main():
    while True:
        user_input = input("Command: ")
        if user_input.lower() in ["exit", "quit"]:
            print("Goodbye!")
            break
        response = agent.run(user_input)
        print(response)

if __name__ == "__main__":
    main()
