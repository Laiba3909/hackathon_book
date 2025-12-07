from fastapi import FastAPI, HTTPException
from fastapi.middleware.cors import CORSMiddleware
from pydantic import BaseModel
from openai import OpenAI
from src.services import get_service_health, qdrant_client_wrapper, OPENAI_API_KEY
from src.embeddings_pipeline.embedder import Embedder
from typing import List

# Initialize FastAPI app (already present)
app = FastAPI()

# Add CORS middleware (already present)
origins = [
    "http://localhost",
    "http://localhost:3000",  # Default Docusaurus port
    # Add other origins as needed for your deployment
]

app.add_middleware(
    CORSMiddleware,
    allow_origins=origins,
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# Initialize Embedder and OpenAI client
try:
    embedder = Embedder()
    openai_client = OpenAI(api_key=OPENAI_API_KEY)
except ValueError as e:
    raise HTTPException(status_code=500, detail=f"Failed to initialize AI components: {e}")

# Define request and response models
class ChatRequest(BaseModel):
    query: str

class ChatResponse(BaseModel):
    response: str
    context: List[dict] = []

@app.get("/health")
def health_check_endpoint():
    """
    Checks the health of the FastAPI service and its dependencies (Qdrant, OpenAI).
    """
    return get_service_health()

@app.post("/chat", response_model=ChatResponse)
async def chat_with_bot(request: ChatRequest):
    """
    Handles chatbot interactions by performing similarity search and generating responses.
    """
    user_query = request.query
    collection_name = "ai_native_textbook_embeddings" # Ensure this matches your pipeline

    try:
        # 1. Generate embedding for the user's query
        query_vector = embedder.get_embedding(user_query)
        if not query_vector:
            raise HTTPException(status_code=500, detail="Could not generate embedding for the query.")

        # 2. Perform similarity search in Qdrant
        search_results = qdrant_client_wrapper.search_vectors(
            collection_name=collection_name,
            query_vector=query_vector,
            limit=3 # Retrieve top 3 relevant chunks
        )

        context = []
        if search_results:
            for hit in search_results:
                context.append({
                    "text": hit.payload.get("text", ""),
                    "source": hit.payload.get("filepath", "unknown"),
                    "section": hit.payload.get("section_title", "unknown")
                })
        
        # 3. Construct a prompt for OpenAI's chat completion with the retrieved context
        context_str = "\n".join([f"Source: {c['source']}, Section: {c['section']}\nContent: {c['text']}" for c in context])
        
        system_message = (
            "You are an AI assistant designed to answer questions about the 'Physical AI & Humanoid Robotics' textbook. "
            "Use the provided context to answer the user's question concisely and accurately. "
            "If the answer is not in the context, state that you don't know. "
            "Do not make up information."
        )
        
        messages = [
            {"role": "system", "content": system_message},
            {"role": "user", "content": f"Context:\n{context_str}\n\nQuestion: {user_query}"}
        ]

        # 4. Call OpenAI's chat completion API
        chat_completion = openai_client.chat.completions.create(
            model="gemini-2.5-flash", # Or "gpt-4", depending on your preference and access
            messages=messages
        )
        
        bot_response = chat_completion.choices[0].message.content

        return ChatResponse(response=bot_response, context=context)

    except HTTPException as e:
        raise e
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"An unexpected error occurred: {e}")
