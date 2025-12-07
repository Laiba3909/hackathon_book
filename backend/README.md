# Backend Integration Stubs

This directory contains placeholder structures for the backend services that will support the AI-Native Textbook. These stubs are intended to define the architectural integration points without providing full implementations.

## fastapi_service

This directory represents the FastAPI service layer. It will house the main application logic, API endpoints, and orchestrate interactions between the frontend, the Qdrant vector database, and the Neon Postgres user store.

## qdrant_connector

This directory will contain the client code and configuration for connecting to and interacting with the Qdrant Cloud vector database. It will handle embedding storage, retrieval, and similarity search operations.

## neon_postgres

This directory is for the Neon Postgres user store. It will define database models, CRUD (Create, Read, Update, Delete) operations, and connection logic for managing user-specific data, personalization settings, and other relational data.