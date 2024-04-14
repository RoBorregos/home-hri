import os
from openai import OpenAI
import base64

IMAGE_PATH = "../gptvision/foto5.png"
MODEL_NAME = "gpt-4-vision-preview"

# Loas client information
client = OpenAI(
    api_key=os.getenv("OPENAI_API_KEY")
)

# Read and encode the image in base64
def read_and_encode_image(image_path):
    
    with open(image_path, "rb") as image_file:
        encoded_image = base64.b64encode(image_file.read()).decode("utf-8")
    
    return encoded_image

# Generate chat response
def generate_chat_response(encoded_image):
    user_message = {
        "role": "user",
        "content": [
            {"type": "text", "text": "You have to name 4 characteristics of the person in the image. The characteristics you have to find are: color of clothes, color of hair, gender and age. Write the characteristics is a short way and write each characteristic on a different line. Organize the information like this example: '- Gender: Female - Age: young adult - Hair color: Blonde - Clothing color: Light blue shirt, white pants' ."},
            {
                "type": "image_url",
                "image_url": f"data:image/jpeg;base64,{encoded_image}"
            },
        ],
    }

    # Send request to OpenAI API
    response = client.chat.completions.create(
        model=MODEL_NAME,
        messages=[user_message],
        max_tokens=50,
    )

    return response.choices[0].message.content



def main():
    # Read and encode the image
    encoded_image = read_and_encode_image(IMAGE_PATH)

    # Generate chat response
    response_content = generate_chat_response(encoded_image)

    print(response_content)

if __name__ == "__main__":
    main()