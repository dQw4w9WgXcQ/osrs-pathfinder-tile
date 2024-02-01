FROM rust as build
WORKDIR /osrs-pathfinder-tile
COPY . .
RUN cargo build --package service --release

FROM ubuntu
COPY --from=build /osrs-pathfinder-tile/target/release/service /app/service
WORKDIR /app
EXPOSE 8080
CMD ["./service"]