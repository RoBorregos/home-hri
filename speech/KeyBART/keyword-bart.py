# pip install --upgrade git+https://github.com/UKPLab/sentence-transformers
# pip install keybert ctransformers
# pip install --upgrade git+https://github.com/huggingface/transformers

from ctransformers import AutoModelForCausalLM

# Set gpu_layers to the number of layers to offload to GPU. 
# Set to 0 if no GPU acceleration is available on your system.
model = AutoModelForCausalLM.from_pretrained(
    "TheBloke/Mistral-7B-Instruct-v0.1-GGUF",
    model_file="mistral-7b-instruct-v0.1.Q4_K_M.gguf",
    model_type="mistral",
    gpu_layers=50,
    hf=True
)

# Generate hugging face access token
# huggingface-cli login

from transformers import AutoTokenizer, pipeline

# Tokenizer
tokenizer = AutoTokenizer.from_pretrained("mistralai/Mistral-7B-Instruct-v0.1")

# Pipeline
generator = pipeline(
    model=model, tokenizer=tokenizer,
    task='text-generation',
    max_new_tokens=50,
    repetition_penalty=1.1,
)


# Generate response
response = generator("What is 1+1?")
print(response[0]["generated_text"])