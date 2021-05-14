import 'package:dorvan/application/constants/phone.dart';
import 'package:dorvan/domain/entity/article.dart';
import 'package:dorvan/domain/entity/chapter.dart';
import 'package:dorvan/presentation/white_button.dart';
import 'package:flutter/material.dart';
import 'package:font_awesome_flutter/font_awesome_flutter.dart';
import 'package:url_launcher/url_launcher.dart';

import 'aware_text.dart';
import 'second_scroll_view.dart';

class ContactView extends StatelessWidget {
  ContactView();
  @override
  Widget build(BuildContext context) {
    return Padding(
      padding: const EdgeInsets.all(30.0),
      child: Stack(
        children: [
          Align(
            alignment: Alignment(-1, -0.3),
            child: Text(
              '05',
              maxLines: 1,
              softWrap: false,
              overflow: TextOverflow.visible,
              style: TextStyle(color: Colors.white30, fontSize: 250),
            ),
          ),
          Align(
            alignment: Alignment(-1, 0.5),
            child: Column(
              mainAxisSize: MainAxisSize.min,
              crossAxisAlignment: CrossAxisAlignment.start,
              children: [
                AwareText(
                  'contact_title',
                  builder: (text) => Text(
                    text,
                    style: TextStyle(color: Colors.white, fontSize: 30),
                  ),
                ),
                SizedBox(height: 20),
                AwareText(
                  'contact_description',
                  builder: (text) => Text(
                    text,
                    style: TextStyle(color: Colors.white60, fontSize: 14),
                  ),
                ),
                SizedBox(height: 30),
                Row(
                  mainAxisAlignment: MainAxisAlignment.spaceEvenly,
                  children: [
                    FaIcon(
                      FontAwesomeIcons.instagram,
                      color: Colors.white,
                    ),
                    FaIcon(
                      FontAwesomeIcons.whatsapp,
                      color: Colors.white,
                    ),
                    FaIcon(
                      FontAwesomeIcons.envelope,
                      color: Colors.white,
                    ),
                  ],
                )
              ],
            ),
          ),
        ],
      ),
    );
  }

  void contact(BuildContext context) async {
    var whatsappUrl = "whatsapp://send?phone=$kPhoneNumber";
    await canLaunch(whatsappUrl)
        ? launch(whatsappUrl)
        : ScaffoldMessenger.of(context).showSnackBar(SnackBar(
            content: Text("Can't launch Whatsapp"),
          ));
  }

  void openInstagram(BuildContext context) {
    final url = 'http:https://www.instagram.com/kendra.str/';
    launch(url).then((value) => null).catchError((e) {
      ScaffoldMessenger.of(context).showSnackBar(SnackBar(
        content: Text("Can't launch Whatsapp"),
      ));
    });
  }
}
