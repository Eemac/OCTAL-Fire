def get_rx_messages(subs, messages):
    """
    Gets the received Messages from the list of messages, based on the subs
    dictionary.
    """
    # Check to see if subs has a message not specified in the DBC
    dbc_message_names = [dbc_message.name for dbc_message in messages]
    masks = None
    rx_messages = list()

    if subs is not None:
        for message in subs:
            if message["name"] not in dbc_message_names:
                raise ValueError(
                    f"Subscription message {message['name']} was not found in the dbc messages."
                )

        rx_message_names = [d["name"] for d in subs]
        rx_messages = list(filter(lambda m: m.name in rx_message_names, messages))
        masks = {message["name"]: message.get("mask", 0x7FF) for message in subs}


    return rx_messages, masks
